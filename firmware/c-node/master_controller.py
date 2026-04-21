#!/usr/bin/env python3
"""
master_controller.py  —  RoboRebound C-Node  |  MASTER (Bot ID = 1)
=====================================================================

Architecture
------------
One master RPi manages N slave RPis over TCP.

  Master (Bot 1)
    - Binds a TCP server on port 9000
    - Accepts one persistent connection per slave
    - Receives full telemetry from every slave (IMU, GPS, vision, state)
    - Maintains a live fleet table: {robot_id -> RobotState}
    - Runs the global swarm decision algorithm
    - Broadcasts a command packet back to ALL slaves simultaneously
    - Also drives its own motors via the local A-Node UART

  Slaves (Bot 2, 3, ...)
    - Connect to master at startup; auto-reconnect on drop
    - Send telemetry to master at 20 Hz
    - Receive a CMD packet from master; execute motor commands locally
    - The slave file is  slave_controller.py

Wire connections (identical on every bot)
------------------------------------------
  /dev/serial0   (GPIO14/15, 115200)  <--  STM32 S-Node UART4
  /dev/ttyAMA1   (GPIO0/1,   115200)  -->  STM32 A-Node UART2

Packet protocol (TCP, newline-delimited JSON)
----------------------------------------------

  Slave -> Master  (TELEM):
    {
      "msg"       : "TELEM",
      "robot_id"  : 3,
      "ts"        : 12500,          # STM32 HAL_GetTick() ms
      "imu" : {
          "ax":0.01,"ay":-0.02,"az":9.81,
          "gx":0.1, "gy":-0.3, "gz":0.0
      },
      "gps" : {
          "lat":39.9526,"lon":-75.1652,"alt":12.3,"fix":1,"sats":7
      },
      "vision" : {
          "line_valid":1,"line_error":-8,"line_angle":0,
          "obs_flag":0,"obs_zone":"N","obs_score":0
      },
      "motors" : {"left":120,"right":120},
      "status"  : "OK"           # OK | OBSTACLE | LOST_LINE | STOPPED
    }

  Master -> All  (CMD):
    {
      "msg"      : "CMD",
      "ts"       : 12550,         # master wall-clock ms
      "targets"  : {
          "1": {"left":120,"right":120,"mode":"LINE_FOLLOW"},
          "3": {"left":0,  "right":0,  "mode":"STOP"},
          "4": {"left":60, "right":60, "mode":"SLOW"}
      },
      "fleet"    : {              # snapshot of all known bot states
          "1": {"lat":...,"lon":...,"obs":0,"status":"OK"},
          "3": {"lat":...,"lon":...,"obs":1,"status":"OBSTACLE"},
          "4": {"lat":...,"lon":...,"obs":0,"status":"OK"}
      }
    }

Usage
-----
  python3 master_controller.py [--bot-id 1] [--port 9000]
                               [--snode-port /dev/serial0]
                               [--anode-port /dev/ttyAMA1]

  Bot-id for the master is always 1 by convention, but can be set.

Dependencies
------------
  pip install pyserial
"""

import argparse
import json
import logging
import socket
import struct
import threading
import time
from dataclasses import dataclass, field, asdict
from typing import Dict, Optional

# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────
MASTER_TCP_PORT   = 9000
UART_BAUD         = 115200
SNODE_PORT_DEF    = '/dev/serial0'
ANODE_PORT_DEF    = '/dev/ttyAMA1'

FRAME_START       = 0x7E
FRAME_END         = 0x7F
HASH_SIZE         = 32

MAX_RPM           = 225
BASE_RPM          = 120
KP_LINE           = 0.5
WATCHDOG_S_NODE   = 2.0    # stop motors if S-Node goes silent
WATCHDOG_SLAVE    = 5.0    # mark slave as LOST if no telem for this long
CMD_BROADCAST_HZ  = 20     # how often master sends CMD to all slaves

PAYLOAD_FMT  = '<I9fH256B BhhBBI'
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FMT)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s'
)
log = logging.getLogger('MASTER')


# ─────────────────────────────────────────────────────────────────────────────
# Data structures
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class IMUData:
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0


@dataclass
class GPSData:
    lat:  float = 0.0
    lon:  float = 0.0
    alt:  float = 0.0
    fix:  int   = 0
    sats: int   = 0


@dataclass
class VisionData:
    line_valid: int = 0
    line_error: int = 0
    line_angle: int = 0
    obs_flag:   int = 0
    obs_zone:   str = 'N'
    obs_score:  int = 0


@dataclass
class MotorState:
    left:  int = 0
    right: int = 0


@dataclass
class RobotState:
    """Live state entry for one robot in the fleet table."""
    robot_id:   int         = 0
    ts:         int         = 0        # last STM32 timestamp ms
    last_seen:  float       = 0.0      # local time.time() of last telem
    imu:        IMUData     = field(default_factory=IMUData)
    gps:        GPSData     = field(default_factory=GPSData)
    vision:     VisionData  = field(default_factory=VisionData)
    motors:     MotorState  = field(default_factory=MotorState)
    status:     str         = 'UNKNOWN'   # OK | OBSTACLE | LOST_LINE | STOPPED | LOST

    def is_alive(self) -> bool:
        return (time.time() - self.last_seen) < WATCHDOG_SLAVE

    def to_fleet_summary(self) -> dict:
        return {
            'robot_id': self.robot_id,
            'lat':      self.gps.lat,
            'lon':      self.gps.lon,
            'alt':      self.gps.alt,
            'fix':      self.gps.fix,
            'ax':       self.imu.ax,
            'ay':       self.imu.ay,
            'az':       self.imu.az,
            'obs':      self.vision.obs_flag,
            'obs_zone': self.vision.obs_zone,
            'line_err': self.vision.line_error,
            'status':   self.status,
            'alive':    self.is_alive(),
        }


def parse_telem(msg: dict) -> RobotState:
    """Parse an incoming TELEM JSON dict into a RobotState."""
    rs = RobotState()
    rs.robot_id  = int(msg.get('robot_id', 0))
    rs.ts        = int(msg.get('ts', 0))
    rs.last_seen = time.time()
    rs.status    = str(msg.get('status', 'OK'))

    imu_d = msg.get('imu', {})
    rs.imu = IMUData(
        ax=float(imu_d.get('ax', 0)),
        ay=float(imu_d.get('ay', 0)),
        az=float(imu_d.get('az', 0)),
        gx=float(imu_d.get('gx', 0)),
        gy=float(imu_d.get('gy', 0)),
        gz=float(imu_d.get('gz', 0)),
    )
    gps_d = msg.get('gps', {})
    rs.gps = GPSData(
        lat =float(gps_d.get('lat',  0)),
        lon =float(gps_d.get('lon',  0)),
        alt =float(gps_d.get('alt',  0)),
        fix =int  (gps_d.get('fix',  0)),
        sats=int  (gps_d.get('sats', 0)),
    )
    vis_d = msg.get('vision', {})
    rs.vision = VisionData(
        line_valid=int(vis_d.get('line_valid', 0)),
        line_error=int(vis_d.get('line_error', 0)),
        line_angle=int(vis_d.get('line_angle', 0)),
        obs_flag  =int(vis_d.get('obs_flag',   0)),
        obs_zone  =str(vis_d.get('obs_zone',  'N')),
        obs_score =int(vis_d.get('obs_score',  0)),
    )
    mot_d = msg.get('motors', {})
    rs.motors = MotorState(
        left =int(mot_d.get('left',  0)),
        right=int(mot_d.get('right', 0)),
    )
    return rs


# ─────────────────────────────────────────────────────────────────────────────
# Frame parser  (binary STM32 S-Node packets -> SensorPayload dict)
# ─────────────────────────────────────────────────────────────────────────────
class FrameParser:
    """State-machine: [0x7E][LEN_H][LEN_L][PAYLOAD+HASH][0x7F]"""
    WAIT_START, LEN_H, LEN_L, DATA, END_BYTE = range(5)

    def __init__(self, ser):
        self.ser   = ser
        self.state = self.WAIT_START
        self.inner_len = 0
        self.buf   = bytearray()
        self._cbs  = []

    def on_frame(self, cb):
        self._cbs.append(cb)

    def run(self):
        while True:
            raw = self.ser.read(1)
            if not raw:
                continue
            b = raw[0]
            if self.state == self.WAIT_START:
                if b == FRAME_START:
                    self.state = self.LEN_H
                    self.buf.clear()
            elif self.state == self.LEN_H:
                self.inner_len = b << 8
                self.state = self.LEN_L
            elif self.state == self.LEN_L:
                self.inner_len |= b
                self.state = self.DATA
            elif self.state == self.DATA:
                self.buf.append(b)
                if len(self.buf) == self.inner_len:
                    self.state = self.END_BYTE
            elif self.state == self.END_BYTE:
                if b == FRAME_END:
                    payload_bytes = bytes(self.buf[:-HASH_SIZE])
                    for cb in self._cbs:
                        try:
                            cb(payload_bytes)
                        except Exception as e:
                            log.error(f"Frame cb error: {e}")
                self.state = self.WAIT_START


def unpack_snode_frame(raw: bytes) -> dict:
    """Unpack binary full_payload_t into a plain dict."""
    if len(raw) < PAYLOAD_SIZE:
        raise ValueError(f"Too short: {len(raw)}")
    f = struct.unpack_from(PAYLOAD_FMT, raw, 0)
    base = 267  # skip 256 img bytes
    return {
        'ts'  : f[0],
        'imu' : {'ax': f[1], 'ay': f[2], 'az': f[3],
                 'gx': f[4], 'gy': f[5], 'gz': f[6]},
        'gps' : {'lat': f[7], 'lon': f[8], 'alt': f[9], 'fix': 0, 'sats': 0},
        'vision': {
            'line_valid': f[base],
            'line_error': f[base+1],
            'line_angle': f[base+2],
            'obs_flag'  : f[base+3],
            'obs_zone'  : chr(f[base+4]) if f[base+4] else 'N',
            'obs_score' : f[base+5],
        }
    }


# ─────────────────────────────────────────────────────────────────────────────
# A-Node interface  (RPi -> STM32 A-Node UART)
# ─────────────────────────────────────────────────────────────────────────────
class ANodeInterface:
    def __init__(self, ser):
        self.ser  = ser
        self.lock = threading.Lock()

    def send_motor(self, left: int, right: int):
        left  = max(-MAX_RPM, min(MAX_RPM, int(left)))
        right = max(-MAX_RPM, min(MAX_RPM, int(right)))
        cmd   = f"MOT,{left},{right}\r\n".encode()
        with self.lock:
            try:
                self.ser.write(cmd)
            except Exception as e:
                log.error(f"[ANode] write error: {e}")

    def stop(self):
        self.send_motor(0, 0)


# ─────────────────────────────────────────────────────────────────────────────
# Fleet registry  (thread-safe)
# ─────────────────────────────────────────────────────────────────────────────
class FleetRegistry:
    """Holds the latest RobotState for every known robot including the master."""

    def __init__(self):
        self._states: Dict[int, RobotState] = {}
        self._lock = threading.Lock()

    def update(self, state: RobotState):
        with self._lock:
            self._states[state.robot_id] = state

    def get(self, robot_id: int) -> Optional[RobotState]:
        with self._lock:
            return self._states.get(robot_id)

    def all_states(self) -> Dict[int, RobotState]:
        with self._lock:
            return dict(self._states)

    def any_obstacle(self, exclude_id: int = -1) -> bool:
        with self._lock:
            return any(
                s.vision.obs_flag and s.robot_id != exclude_id
                for s in self._states.values()
                if s.is_alive()
            )

    def alive_count(self) -> int:
        with self._lock:
            return sum(1 for s in self._states.values() if s.is_alive())


# ─────────────────────────────────────────────────────────────────────────────
# Swarm decision engine  (runs on master only)
# ─────────────────────────────────────────────────────────────────────────────
class SwarmDecisionEngine:
    """
    Global coordinator. Reads the fleet table every control tick and
    produces a targets dict:  {robot_id_str -> {left, right, mode}}
    """

    def __init__(self, master_id: int, fleet: FleetRegistry):
        self.master_id = master_id
        self.fleet     = fleet

    def compute_targets(self) -> dict:
        """
        Returns a dict keyed by str(robot_id) with motor commands.

        Rules (priority order):
          1. If a robot is LOST (no telem), skip it.
          2. If the robot itself has an obstacle  -> STOP that robot.
          3. If ANY other alive robot has obstacle -> slow ALL others to 40%.
          4. If the robot's line is valid          -> proportional line-follow.
          5. Otherwise                             -> slow rotate-to-search.
        """
        states  = self.fleet.all_states()
        targets = {}

        # Is any robot blocked right now?
        any_blocked = any(
            s.vision.obs_flag
            for s in states.values()
            if s.is_alive()
        )

        for rid, s in states.items():
            if not s.is_alive():
                continue   # don't send stale commands

            rid_str = str(rid)

            if s.vision.obs_flag:
                targets[rid_str] = {'left': 0, 'right': 0, 'mode': 'STOP'}

            elif any_blocked:
                # Slow down — someone in fleet is blocked
                slow = int(BASE_RPM * 0.4)
                targets[rid_str] = {'left': slow, 'right': slow, 'mode': 'SLOW'}

            elif s.vision.line_valid:
                corr  = int(KP_LINE * s.vision.line_error)
                left  = max(0, min(MAX_RPM, BASE_RPM - corr))
                right = max(0, min(MAX_RPM, BASE_RPM + corr))
                targets[rid_str] = {'left': left, 'right': right, 'mode': 'LINE_FOLLOW'}

            else:
                # Rotate to search for line
                targets[rid_str] = {'left': 40, 'right': -40, 'mode': 'SEARCH'}

        return targets


# ─────────────────────────────────────────────────────────────────────────────
# TCP server  (one persistent connection per slave)
# ─────────────────────────────────────────────────────────────────────────────
class MasterTCPServer:
    """
    Binds on MASTER_TCP_PORT. Spawns one thread per connected slave.
    Each thread:
      - Reads TELEM JSON lines from that slave
      - Updates the fleet registry
    A separate broadcast thread pushes CMD JSON to all connected slaves.
    """

    def __init__(self, port: int, fleet: FleetRegistry, engine: SwarmDecisionEngine):
        self.port     = port
        self.fleet    = fleet
        self.engine   = engine
        self._conns: Dict[int, socket.socket] = {}  # robot_id -> socket
        self._lock    = threading.Lock()

    def start(self):
        t_accept    = threading.Thread(target=self._accept_loop, daemon=True)
        t_broadcast = threading.Thread(target=self._broadcast_loop, daemon=True)
        t_accept.start()
        t_broadcast.start()
        log.info(f"[Server] TCP server started on port {self.port}")

    def _accept_loop(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('0.0.0.0', self.port))
        srv.listen(10)
        while True:
            conn, addr = srv.accept()
            log.info(f"[Server] New connection from {addr}")
            t = threading.Thread(
                target=self._slave_handler, args=(conn, addr), daemon=True
            )
            t.start()

    def _slave_handler(self, conn: socket.socket, addr):
        """
        Each slave sends its robot_id in the very first TELEM message.
        We register the connection and route all subsequent telemetry.
        """
        robot_id = None
        buf = b''
        try:
            conn.settimeout(10.0)
            while True:
                chunk = conn.recv(1024)
                if not chunk:
                    break
                buf += chunk
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        msg = json.loads(line.decode())
                    except Exception:
                        continue

                    if msg.get('msg') == 'TELEM':
                        state = parse_telem(msg)

                        # Register socket the first time we learn the robot_id
                        if robot_id is None:
                            robot_id = state.robot_id
                            with self._lock:
                                self._conns[robot_id] = conn
                            log.info(f"[Server] Slave robot_id={robot_id} registered")

                        self.fleet.update(state)
                        log.debug(
                            f"[Server] TELEM robot={state.robot_id} "
                            f"lat={state.gps.lat:.4f} lon={state.gps.lon:.4f} "
                            f"obs={state.vision.obs_flag} status={state.status}"
                        )
        except Exception as e:
            log.warning(f"[Server] Slave {addr} error: {e}")
        finally:
            if robot_id is not None:
                with self._lock:
                    self._conns.pop(robot_id, None)
                log.info(f"[Server] Slave robot_id={robot_id} disconnected")
            conn.close()

    def _broadcast_loop(self):
        """Push CMD packet to every connected slave at CMD_BROADCAST_HZ."""
        interval = 1.0 / CMD_BROADCAST_HZ
        while True:
            time.sleep(interval)
            try:
                targets  = self.engine.compute_targets()
                all_st   = self.fleet.all_states()
                fleet_snap = {
                    str(rid): s.to_fleet_summary()
                    for rid, s in all_st.items()
                }
                cmd = {
                    'msg'    : 'CMD',
                    'ts'     : int(time.time() * 1000),
                    'targets': targets,
                    'fleet'  : fleet_snap,
                }
                payload = (json.dumps(cmd) + '\n').encode()

                with self._lock:
                    dead = []
                    for rid, conn in self._conns.items():
                        try:
                            conn.sendall(payload)
                        except Exception as e:
                            log.warning(f"[Server] CMD send to {rid} failed: {e}")
                            dead.append(rid)
                    for rid in dead:
                        self._conns.pop(rid, None)
            except Exception as e:
                log.error(f"[Server] Broadcast error: {e}")


# ─────────────────────────────────────────────────────────────────────────────
# Master local control loop  (drives its own motors)
# ─────────────────────────────────────────────────────────────────────────────
class MasterLocalController:
    """
    The master bot also has its own S-Node and A-Node.
    This runs the control loop for the master's own motors based on its
    own sensor data and the global fleet state.
    """

    def __init__(self, master_id: int, anode: ANodeInterface,
                 fleet: FleetRegistry, engine: SwarmDecisionEngine):
        self.master_id = master_id
        self.anode     = anode
        self.fleet     = fleet
        self.engine    = engine
        self._my_raw   = {}           # latest unpacked snode frame dict
        self._raw_lock = threading.Lock()
        self._last_frame_t = time.time()

    def on_snode_frame(self, frame_dict: dict):
        """Called from FrameParser callback — updates master's own fleet entry."""
        with self._raw_lock:
            self._my_raw = frame_dict
            self._last_frame_t = time.time()

        # Build a RobotState for the master itself and push to fleet
        rs = RobotState()
        rs.robot_id  = self.master_id
        rs.ts        = frame_dict.get('ts', 0)
        rs.last_seen = time.time()

        imu_d = frame_dict.get('imu', {})
        rs.imu = IMUData(
            ax=imu_d.get('ax', 0), ay=imu_d.get('ay', 0), az=imu_d.get('az', 0),
            gx=imu_d.get('gx', 0), gy=imu_d.get('gy', 0), gz=imu_d.get('gz', 0),
        )
        gps_d = frame_dict.get('gps', {})
        rs.gps = GPSData(
            lat=gps_d.get('lat', 0), lon=gps_d.get('lon', 0),
            alt=gps_d.get('alt', 0), fix=gps_d.get('fix', 0),
        )
        vis_d = frame_dict.get('vision', {})
        rs.vision = VisionData(
            line_valid=vis_d.get('line_valid', 0),
            line_error=vis_d.get('line_error', 0),
            line_angle=vis_d.get('line_angle', 0),
            obs_flag  =vis_d.get('obs_flag',   0),
            obs_zone  =vis_d.get('obs_zone',  'N'),
            obs_score =vis_d.get('obs_score',  0),
        )
        rs.status = 'OBSTACLE' if rs.vision.obs_flag else (
                    'LOST_LINE' if not rs.vision.line_valid else 'OK')
        self.fleet.update(rs)

    def run(self):
        """Main thread — applies computed targets to the master's own motors."""
        log.info(f"[Master-Local] Bot {self.master_id} motor loop starting")
        while True:
            # Watchdog: if S-Node is silent, stop
            with self._raw_lock:
                last_t = self._last_frame_t

            if time.time() - last_t > WATCHDOG_S_NODE:
                log.warning("[Master-Local] S-Node timeout — stopping")
                self.anode.stop()
                time.sleep(0.1)
                continue

            targets = self.engine.compute_targets()
            my_target = targets.get(str(self.master_id))
            if my_target:
                self.anode.send_motor(my_target['left'], my_target['right'])
                log.debug(f"[Master-Local] motors L={my_target['left']} R={my_target['right']}"
                          f" mode={my_target['mode']}")
            else:
                self.anode.stop()

            time.sleep(1.0 / CMD_BROADCAST_HZ)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description='RoboRebound MASTER C-Node')
    ap.add_argument('--bot-id',      type=int, default=1,
                    help='Robot ID for this master (default: 1)')
    ap.add_argument('--port',        type=int, default=MASTER_TCP_PORT,
                    help='TCP port to listen on (default: 9000)')
    ap.add_argument('--snode-port',  default=SNODE_PORT_DEF)
    ap.add_argument('--anode-port',  default=ANODE_PORT_DEF)
    ap.add_argument('--log-level',   default='INFO',
                    choices=['DEBUG','INFO','WARNING'])
    args = ap.parse_args()

    logging.getLogger().setLevel(args.log_level)
    log.info(f"=== RoboRebound MASTER  bot_id={args.bot_id}  port={args.port} ===")

    import serial

    # Open UARTs
    ser_snode = serial.Serial(args.snode_port, UART_BAUD, timeout=0.05)
    ser_anode = serial.Serial(args.anode_port, UART_BAUD, timeout=0.05)

    # Core objects
    fleet   = FleetRegistry()
    anode   = ANodeInterface(ser_anode)
    engine  = SwarmDecisionEngine(args.bot_id, fleet)
    server  = MasterTCPServer(args.port, fleet, engine)
    local_ctrl = MasterLocalController(args.bot_id, anode, fleet, engine)

    # S-Node frame parser
    frame_parser = FrameParser(ser_snode)
    def on_frame(raw_bytes: bytes):
        try:
            d = unpack_snode_frame(raw_bytes)
            local_ctrl.on_snode_frame(d)
        except Exception as e:
            log.warning(f"[Frame] parse error: {e}")
    frame_parser.on_frame(on_frame)

    # Start threads
    server.start()
    t_frame = threading.Thread(target=frame_parser.run, daemon=True)
    t_frame.start()

    log.info("[Main] All threads started. Waiting for slaves to connect...")
    log.info(f"[Main] Slaves should connect to: <this-ip>:{args.port}")

    # Local motor loop in main thread
    local_ctrl.run()


if __name__ == '__main__':
    main()
