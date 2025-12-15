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