# quadcopter
Quadcopter X project

Quadcopter module's code is mainly based on baselsw's BlueCopter code.

quadcopter - onboard module<br>
xinput_arduino - on ground module used as radio transmitter<br>
xinput - windows program which connects X360 controller with COM port. Has ability to calibrate PIDs in-flight.

2x Arduino UNO<br>
2x nRF24l01<br>
GY-88 9-DOF (MPU6050, HMC5883L, BMP085) [code now uses only MPU]<br>
X360 Controller for Windows <br>

Currently not yet flying, probably beacuse of exponential potentiometer in X360 controller's trigger (my QC has hover point at about 50% of throttle) so there is no precision of throttle when i'm trying to regulate engines power.
