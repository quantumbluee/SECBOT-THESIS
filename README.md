<<<<<<< HEAD
# SecBot 
## Overview:

SecBot is a distrubuted sensing/control architecture inspired by the design in the original thesis paper (2024) and the RoboRebound research paper (EuroSys 2025). The system consists of three hardware nodes that communicat to perform sensing, vision inference, and actuation:  

---
## 1. S-Node (Sensor Node)  
Runs on **STM32F723E-Discovery (Cortex-M7)** and performs:  
- GPS acquisition (NMEA GGA sentences via USART6)
- IMU sensing using **ICM-20948** over I2C2 
- Vision metadata/streaming from **OpenMV H7 R2** over UART7
- Timestamped fused payload generation
- SHA-256 hashing of payloads
- Binary **framing protocol** used to send data to C-node  

The S-Node is responsible for collecting all raw sensor data.  

---
## 2. C-Node (Control Node/ Edge Processor)  
Runs on a **Raspberry Pi 5** and handles:  
- Receiving and parsing S-Node framed packets  
- Verifying SHA-256 hashes
- Running computer-vision inference (OpenCV, TFLite)
- Synchronizing IMU + GPS + Vision streams
- Making higher-level decisions based on sensor fusion
- Sending commands to the A-Node (actuation)  

The C-Node is the main controller of the system/  

---
## 3. A-Node (Actuator Node)  
Runs on **STM32 motor controller**. Its tasks: 
- Receive commands from the C-Node
- Generate PWM / direction control for motors  
- Optional feedback  

---  
# S-Node Payload Format  
The S-Node produces structures payloads:
```c
typedef struct {
    uint32_t timestamp_ms;

    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;

    float latitude, longitude, altitude;

    uint16_t img_size;
    uint8_t img_data[256];
}s_node_payload_t;
```  

# Framing Protocol (S-Node --> C-Node)
```[0x7E][LEN_H][LEN_L][PAYLOAD][SHA256 (32 bytes)][0x7F]``` 
   
Documented in ```docs/protocol-spec.md```

# C-Node Vision Pipeline  
-- TO BE UPDATED
=======
# security_secbot
The hardware platform for secure multi-robot systems

## A-Node Motor Controller

Motor control firmware for RoboRebound A-Node. Controls DC motor via L298N driver with alternating speeds and directions.

### Motor Behavior
- Runs at **100 RPM** (direction 1) for 10 seconds
- Switches to **170 RPM** (direction 2) for 10 seconds
- Repeats continuously

### Hardware Connections
| STM32 Pin | L298N Pin | Function |
|-----------|-----------|----------|
| PA5 | IN1 | PWM Control (TIM2 CH1) |
| PA6 | IN2 | PWM Control (TIM3 CH1) |
| GND | GND | Common ground |

**Power:** 9V battery to L298N (motor max speed = 225 RPM @ 9V)

### Code Structure
- **Bare-metal register programming** (no HAL, no .ioc file)
- PWM frequency: 50 Hz (20ms period)
- Uses TIM2/TIM3 for PWM, TIM4 for delays

### Customization
Change speeds in `main()`:
```c
Motor_Control(1, 100);  // Change RPM (0-225)
TIM4_ms_Delay(10000);   // Change delay (ms)
```
>>>>>>> ab8d6f16a5b891d2497e2ac95cfd5e0a9d560335
