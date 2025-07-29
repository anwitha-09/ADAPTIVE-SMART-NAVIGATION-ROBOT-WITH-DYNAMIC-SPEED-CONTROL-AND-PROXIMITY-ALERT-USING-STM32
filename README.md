# Adaptive Smart Navigation Robot (STM32F407VG)

This project implements a smart obstacle-avoiding robot using the STM32F407VG microcontroller. The robot uses three ultrasonic sensors to detect obstacles and dynamically adjusts its speed and direction. It provides both audio and visual proximity alerts, making it suitable for smart navigation applications in indoor environments.

## Features

- Real-time obstacle detection using front, rear-left, and rear-right ultrasonic sensors
- Adaptive navigation: forward, reverse, left, right, stop
- Dynamic speed control using PWM (TIM4 channels)
- Buzzer and LEDs for proximity alerts
- Accurate ultrasonic distance measurement with DWT-based microsecond delays
- Well-structured and modular embedded C code for easy customization

## Hardware Used

- STM32F407VGT6 Development Board
- L298N Dual H-Bridge Motor Driver
- 2 × DC Motors
- 3 × HC-SR04 Ultrasonic Sensors
- Buzzer for audio alerts
- LEDs for visual alerts
- External battery (7–12V recommended)
- (Optional) HC-05 Bluetooth Module for wireless control

## STM32 Pin Mapping

| Function            | STM32 Pin     |
|---------------------|---------------|
| ENA (Motor A)       | PB7 (TIM4_CH2)|
| ENB (Motor B)       | PB6 (TIM4_CH1)|
| IN1 / IN2           | PB1 / PA3     |
| IN3 / IN4           | PB0 / PC4     |
| Front Sensor TRIG   | PE11          |
| Front Sensor ECHO   | PE9           |
| Left Sensor TRIG    | PD1           |
| Left Sensor ECHO    | PD2           |
| Right Sensor TRIG   | PD8           |
| Right Sensor ECHO   | PD9           |
| Buzzer              | PC8           |
| LEDs                | PD12–PD15     |

## How It Works

1. On startup, the robot briefly moves forward to initialize motors.
2. In the main loop:
   - All three ultrasonic sensors continuously measure distances.
   - If an obstacle is too close:
     - LEDs light up and the buzzer sounds.
     - The robot evaluates direction:
       - If front is blocked: turn left or right based on rear sensor data.
       - If all directions blocked: reverse.
       - Otherwise: move forward.
   - Motor speed is adjusted dynamically:
     - Lower speed if obstacles are near.
     - Higher speed if the path is clear.
