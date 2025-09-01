# ESP32-Drone


---
## Software
The code is structurd in modules:

**main**  
- Init: SBUS, IMU, PWM, LED, Battery  
- Safety: Failsafe, Low-voltage, Arming  
- Motor mix: Inputs + PID corrections  
**SbusReceiver**  
- Reads SBUS signal  
- Maps + filters channels  
- Deadband, scaling, failsafe check  

**MPU6050**  
- Reads gyro + accel  
- Complementary filter for roll/pitch  
- Provides orientation data  

**PidController**  
- Generic PID loop  
- Used for roll and pitch stabilization  (no yaw!)

**FlightController**  
- Reads roll/pitch targets from SBUS and actual values from IMU  
- Runs PID controllers for roll and pitch  
- Outputs correction values (but the Donelogic is used in main yet.)

**config.h**  
- Pin definitions  
- PID parameters  
- PWM setup  
- Battery thresholds 

---
## Hardware

**Supported Hardware:  
- ESP32-C3  
- MPU6050 // MPU6500  
- SBUS Receiver (serial data from RadioLink remote controller)  
- 4 × Brushed DC Motors (3.7 V)  
- Custom Motor Drivers (MOSFET + gate resistor + pull-down + Schottky diode)  
- 1S LiPo Battery  
- Resistor Divider (for battery voltage measurement)  
- LED Indicator (low-voltage warning)  
- Capacitors for voltage spikes  

The system is based on an ESP32-C3 Super Mini microcontroller combined with an MPU6500 IMU for orientation sensing.  
Remote control signals are received by a serial SBUS input.  
Four DC brushed motors are controlled by custom motor drivers. Each is build with a Mosfet, pulldown resisitor and a shottky diode. 

## Project Images
<img src="https://github.com/user-attachments/assets/04512347-8bc8-4630-acfe-72a15ddec1d4" width="50%" /> 
