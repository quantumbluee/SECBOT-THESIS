#!/usr/bin/env python3
"""
slave_controller.py  —  RoboRebound C-Node  |  SLAVE (Bot ID = 2, 3, 4 ...)
=============================================================================

Each slave:
  1. Reads framed binary packets from its own STM32 S-Node over UART
     (IMU, GPS, vision LINE/OBS data bundled in full_payload_t)
  2. Connects to the master RPi over TCP (auto-reconnects on drop)
  3. Sends a TELEM JSON packet to the master at 20 Hz
     — includes robot_id, IMU, GPS, vision data, current motor state, status
  4. Receives CMD JSON from the master; applies the motor target for this
     robot_id to its own A-Node over UART
  5. Safety: if master connection is lost, falls back to autonomous
     line-following until reconnected

Wire connections (identical to master)
----------------------------------------
  /dev/serial0   (GPIO14/15, 115200)  <--  STM32 S-Node UART4
  /dev/ttyAMA1   (GPIO0/1,   115200)  -->  STM32 A-Node UART2

Usage
-----
  python3 slave_controller.py --bot-id 2 --master-ip 192.168.1.100
  python3 slave_controller.py --bot-id 3 --master-ip 192.168.1.100
  python3 slave_controller.py --bot-id 4 --master-ip 192.168.1.100

  --bot-id   : must be unique across all bots; master is always bot-id 1
  --master-ip: IP address of the master RPi

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
from dataclasses import dataclass, field

# ─────────────────────────────────────────────────────────────────────────────
# Constants  (must match master_controller.py)
# ─────────────────────────────────────────────────────────────────────────────
MASTER_TCP_PORT  = 9000
UART_BAUD        = 115200
SNODE_PORT_DEF   = '/dev/serial0'
ANODE_PORT_DEF   = '/dev/ttyAMA1'

FRAME_START      = 0x7E
FRAME_END        = 0x7F
HASH_SIZE        = 32

MAX_RPM          = 225
BASE_RPM         = 120
KP_LINE          = 0.5

WATCHDOG_SNODE   = 2.0   # stop if S-Node silent for this long
WATCHDOG_MASTER  = 3.0   # fall back to autonomous if master silent for this long
TELEM_HZ         = 20    # how often to send telemetry to master

PAYLOAD_FMT  = '<I9fH256B BhhBBI'
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FMT)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s'
)
log = logging.getLogger('SLAVE')


# ─────────────────────────────────────────────────────────────────────────────
# Lightweight sensor data holders
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
class SNodeFrame:
    """Parsed result of one binary S-Node frame."""
    ts:     int        = 0
    imu:    IMUData    = field(default_factory=IMUData)
    gps:    GPSData    = field(default_factory=GPSData)
    vision: VisionData = field(default_factory=VisionData)


# ─────────────────────────────────────────────────────────────────────────────
# Frame parser  (binary -> SNodeFrame)
# ─────────────────────────────────────────────────────────────────────────────
class FrameParser:
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


def unpack_frame(raw: bytes) -> SNodeFrame:
    if len(raw) < PAYLOAD_SIZE:
        raise ValueError(f"Payload too short: {len(raw)}")
    f = struct.unpack_from(PAYLOAD_FMT, raw, 0)
    base = 267  # skip 256 image bytes (fields 11..266)
    frame = SNodeFrame()
    frame.ts   = f[0]
    frame.imu  = IMUData(
        ax=f[1], ay=f[2], az=f[3],
        gx=f[4], gy=f[5], gz=f[6],
    )
    frame.gps  = GPSData(
        lat=f[7], lon=f[8], alt=f[9],
        fix=0, sats=0   # GPS fix flag not in binary struct; extend if needed
    )
    frame.vision = VisionData(
        line_valid=f[base],
        line_error=f[base+1],
        line_angle=f[base+2],
        obs_flag  =f[base+3],
        obs_zone  =chr(f[base+4]) if f[base+4] else 'N',
        obs_score =f[base+5],
    )
    return frame


# ─────────────────────────────────────────────────────────────────────────────
# A-Node interface
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
                log.error(f"[ANode] write: {e}")

    def stop(self):
        self.send_motor(0, 0)


# ─────────────────────────────────────────────────────────────────────────────
# Master connection  (persistent TCP client, auto-reconnect)
# ─────────────────────────────────────────────────────────────────────────────
class MasterConnection:
    """
    Maintains a persistent TCP connection to the master RPi.
    - _tx_loop    : sends TELEM JSON at TELEM_HZ
    - _rx_loop    : receives CMD JSON and updates self.latest_cmd
    Both loops reconnect automatically if the connection drops.
    """

    def __init__(self, robot_id: int, master_ip: str, port: int = MASTER_TCP_PORT):
        self.robot_id  = robot_id
        self.master_ip = master_ip
        self.port      = port
        self._conn     = None
        self._conn_lock = threading.Lock()
        self.latest_cmd: dict = {}     # most recent CMD from master
        self._cmd_lock = threading.Lock()
        self._last_cmd_t = 0.0
        self._latest_telem_fn = None   # callable () -> dict, set by SlaveController

    def register_telem_source(self, fn):
        """fn should return the current TELEM dict when called."""
        self._latest_telem_fn = fn

    def start(self):
        threading.Thread(target=self._connection_manager, daemon=True).start()

    def is_connected(self) -> bool:
        with self._conn_lock:
            return self._conn is not None

    def master_is_alive(self) -> bool:
        return self.is_connected() and (time.time() - self._last_cmd_t < WATCHDOG_MASTER)

    def get_cmd(self) -> dict:
        with self._cmd_lock:
            return dict(self.latest_cmd)

    # ── Internal ────────────────────────────────────────────────────────────
    def _connection_manager(self):
        """Opens socket, spawns TX/RX threads, waits, repeats on failure."""
        while True:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(5.0)
                s.connect((self.master_ip, self.port))
                s.settimeout(None)
                log.info(f"[Master] Connected to {self.master_ip}:{self.port}")
                with self._conn_lock:
                    self._conn = s

                done = threading.Event()
                t_tx = threading.Thread(target=self._tx_loop, args=(s, done), daemon=True)
                t_rx = threading.Thread(target=self._rx_loop, args=(s, done), daemon=True)
                t_tx.start()
                t_rx.start()
                done.wait()   # blocks until one thread sets the event

            except Exception as e:
                log.warning(f"[Master] Connection failed: {e}. Retrying in 2s")
            finally:
                with self._conn_lock:
                    self._conn = None
            time.sleep(2)

    def _tx_loop(self, sock: socket.socket, done: threading.Event):
        """Send TELEM to master at TELEM_HZ."""
        interval = 1.0 / TELEM_HZ
        while not done.is_set():
            try:
                if self._latest_telem_fn:
                    telem = self._latest_telem_fn()
                    payload = (json.dumps(telem) + '\n').encode()
                    sock.sendall(payload)
            except Exception as e:
                log.warning(f"[Master] TX error: {e}")
                done.set()
                return
            time.sleep(interval)

    def _rx_loop(self, sock: socket.socket, done: threading.Event):
        """Receive CMD from master."""
        buf = b''
        try:
            sock.settimeout(5.0)
            while not done.is_set():
                try:
                    chunk = sock.recv(2048)
                    if not chunk:
                        log.warning("[Master] Connection closed by master")
                        done.set()
                        return
                    buf += chunk
                    while b'\n' in buf:
                        line, buf = buf.split(b'\n', 1)
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            msg = json.loads(line.decode())
                            if msg.get('msg') == 'CMD':
                                with self._cmd_lock:
                                    self.latest_cmd = msg
                                self._last_cmd_t = time.time()
                                log.debug(f"[Master] CMD received ts={msg.get('ts')}")
                        except json.JSONDecodeError:
                            pass
                except socket.timeout:
                    # No data — check if connection is still expected alive
                    continue
        except Exception as e:
            log.warning(f"[Master] RX error: {e}")
            done.set()


# ─────────────────────────────────────────────────────────────────────────────
# Slave controller  (main loop)
# ─────────────────────────────────────────────────────────────────────────────
class SlaveController:
    """
    Integrates the S-Node frame stream, the master connection, and the
    A-Node motor interface into a single control loop.
    """

    def __init__(self, robot_id: int, anode: ANodeInterface,
                 master: MasterConnection):
        self.robot_id = robot_id
        self.anode    = anode
        self.master   = master

        # Latest sensor data (updated from FrameParser callback thread)
        self._frame   = SNodeFrame()
        self._frame_lock = threading.Lock()
        self._last_frame_t = 0.0

        # Current motor state (for inclusion in TELEM)
        self._motor_left  = 0
        self._motor_right = 0
        self._mode        = 'INIT'

        # Give the master connection a way to get the latest TELEM
        self.master.register_telem_source(self._build_telem)

    # ── Sensor update ────────────────────────────────────────────────────────
    def on_snode_frame(self, raw_bytes: bytes):
        try:
            frame = unpack_frame(raw_bytes)
            with self._frame_lock:
                self._frame = frame
                self._last_frame_t = time.time()
        except Exception as e:
            log.warning(f"[Sensor] parse error: {e}")

    # ── Telemetry builder ────────────────────────────────────────────────────
    def _build_telem(self) -> dict:
        with self._frame_lock:
            f = self._frame

        vis = f.vision
        status = ('OBSTACLE'  if vis.obs_flag        else
                  'LOST_LINE' if not vis.line_valid   else 'OK')

        return {
            'msg'      : 'TELEM',
            'robot_id' : self.robot_id,
            'ts'       : f.ts,
            'imu' : {
                'ax': round(f.imu.ax, 4), 'ay': round(f.imu.ay, 4),
                'az': round(f.imu.az, 4),
                'gx': round(f.imu.gx, 4), 'gy': round(f.imu.gy, 4),
                'gz': round(f.imu.gz, 4),
            },
            'gps' : {
                'lat' : round(f.gps.lat,  6),
                'lon' : round(f.gps.lon,  6),
                'alt' : round(f.gps.alt,  2),
                'fix' : f.gps.fix,
                'sats': f.gps.sats,
            },
            'vision' : {
                'line_valid': vis.line_valid,
                'line_error': vis.line_error,
                'line_angle': vis.line_angle,
                'obs_flag'  : vis.obs_flag,
                'obs_zone'  : vis.obs_zone,
                'obs_score' : vis.obs_score,
            },
            'motors' : {
                'left' : self._motor_left,
                'right': self._motor_right,
            },
            'status': status,
        }

    # ── Autonomous fallback (no master) ──────────────────────────────────────
    def _autonomous_control(self, frame: SNodeFrame) -> tuple:
        """
        Simple local line-follow used when master connection is lost.
        Returns (left_rpm, right_rpm, mode_str).
        """
        vis = frame.vision
        if vis.obs_flag:
            return 0, 0, 'STOP'
        if vis.line_valid:
            corr  = int(KP_LINE * vis.line_error)
            left  = max(0, min(MAX_RPM, BASE_RPM - corr))
            right = max(0, min(MAX_RPM, BASE_RPM + corr))
            return left, right, 'AUTONOMOUS'
        return 40, -40, 'SEARCH'

    # ── Main control loop ─────────────────────────────────────────────────────
    def run(self):
        log.info(f"[Ctrl] Slave robot_id={self.robot_id} control loop starting")
        while True:
            # ── Watchdog: S-Node silent? ─────────────────────────────────
            with self._frame_lock:
                last_t = self._last_frame_t
                frame  = self._frame

            if time.time() - last_t > WATCHDOG_SNODE:
                log.warning("[Ctrl] S-Node timeout — stopping")
                self.anode.stop()
                self._motor_left = self._motor_right = 0
                self._mode = 'STOPPED'
                time.sleep(0.1)
                continue

            # ── Get motor targets ─────────────────────────────────────────
            if self.master.master_is_alive():
                # Master is alive — apply its command for this robot_id
                cmd = self.master.get_cmd()
                targets = cmd.get('targets', {})
                my_target = targets.get(str(self.robot_id))

                if my_target:
                    left  = int(my_target.get('left',  0))
                    right = int(my_target.get('right', 0))
                    mode  = str(my_target.get('mode', 'CMD'))
                else:
                    # Master sent a CMD but didn't include us — hold still
                    left, right, mode = 0, 0, 'HOLD'

                # Log the fleet snapshot if available (useful for debugging)
                fleet = cmd.get('fleet', {})
                if fleet:
                    log.debug(f"[Ctrl] Fleet: " +
                              " | ".join(f"bot{k}:{v.get('status','?')}"
                                         for k, v in fleet.items()))
            else:
                # Master unreachable — autonomous fallback
                if self.master.is_connected():
                    log.warning("[Ctrl] Master connected but CMD stale — autonomous")
                else:
                    log.warning("[Ctrl] Master disconnected — autonomous")
                left, right, mode = self._autonomous_control(frame)

            # ── Apply to A-Node ───────────────────────────────────────────
            self.anode.send_motor(left, right)
            self._motor_left  = left
            self._motor_right = right
            self._mode        = mode

            log.debug(f"[Ctrl] L={left:4d} R={right:4d} mode={mode:12s} "
                      f"line={frame.vision.line_valid}/{frame.vision.line_error:+d} "
                      f"obs={frame.vision.obs_flag}")

            time.sleep(1.0 / TELEM_HZ)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description='RoboRebound SLAVE C-Node')
    ap.add_argument('--bot-id',     type=int, required=True,
                    help='Unique robot ID for this slave (e.g. 2, 3, 4 ...)')
    ap.add_argument('--master-ip',  type=str, required=True,
                    help='IP address of the master RPi')
    ap.add_argument('--port',       type=int, default=MASTER_TCP_PORT,
                    help='Master TCP port (default: 9000)')
    ap.add_argument('--snode-port', default=SNODE_PORT_DEF)
    ap.add_argument('--anode-port', default=ANODE_PORT_DEF)
    ap.add_argument('--log-level',  default='INFO',
                    choices=['DEBUG','INFO','WARNING'])
    args = ap.parse_args()

    logging.getLogger().setLevel(args.log_level)
    log.info(f"=== RoboRebound SLAVE  bot_id={args.bot_id}  master={args.master_ip}:{args.port} ===")

    import serial

    # Open UARTs
    ser_snode = serial.Serial(args.snode_port, UART_BAUD, timeout=0.05)
    ser_anode = serial.Serial(args.anode_port, UART_BAUD, timeout=0.05)

    # Core objects
    anode  = ANodeInterface(ser_anode)
    master = MasterConnection(args.bot_id, args.master_ip, args.port)
    ctrl   = SlaveController(args.bot_id, anode, master)

    # Wire S-Node frames into the controller
    frame_parser = FrameParser(ser_snode)
    frame_parser.on_frame(ctrl.on_snode_frame)

    # Start threads
    master.start()
    threading.Thread(target=frame_parser.run, daemon=True).start()

    log.info(f"[Main] Bot {args.bot_id} started. Connecting to master at {args.master_ip}...")

    # Control loop in main thread
    ctrl.run()


if __name__ == '__main__':
    main()
