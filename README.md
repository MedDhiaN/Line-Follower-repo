# Line Follower Robot
A PID-controlled line follower robot built using PlatformIO and Arduino C++. The repository includes both firmware logic and mechanical design CAD files.
```
Line-Follower-repo/
├── Line Follower Conception/   # 3D mechanical designs & SolidWorks files (.SLDPRT, .EASM)
├── include/                   # Header files
├── src/
│   └── main.cpp               # Core firmware (PID control, sensor readings, motor drive)
├── test/                      # Unit testing scripts
└── platformio.ini             # PlatformIO project configuration
```
# Hardware Setup
IR Sensors: 6-sensor array connected to analog pins A0 – A5

Motors: Differential drive configured on PWM/Digital pins:
```
LEFT_MOTOR_FWD: Pin 11

LEFT_MOTOR_BWD: Pin 10

RIGHT_MOTOR_FWD: Pin 6

RIGHT_MOTOR_BWD: Pin 9
```

# Features & Code Overview (main.cpp)
- Sensor Calibration (calibrate): Reads baseline black and white surface threshold levels across all 6 sensors.

- PID Control (calculatePID): Smooth, responsive tracking using weighted sensor values.

- Motor Control (driveMotors): Sets differential motor speeds based on PID output.

- Line Inversion Support (invertedReadsensors): Adapts readings for dark-on-light or light-on-dark tracks.

# How to Build & Flash
Install VS Code and the PlatformIO IDE extension.

Clone this repository:

```Bash
git clone [https://github.com/MedDhiaN/Line-Follower-repo.git](https://github.com/MedDhiaN/Line-Follower-repo.git)
```

Open the project folder in PlatformIO.
Connect your microcontroller via USB.
Click Build and Upload.
