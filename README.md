# 5DOF-robotic-arm-project
Learning embedded systems through robotic arm
## Part 1: 
# LED Brightness Control via Joystick

Learning project following guided tutorial. Code written with AI assistance to understand PWM and ADC concepts.

Focus was on understanding:
- Basic electronics/breadboard usage
- How PWM controls brightness
- ADC voltage conversion

Overview: learned breadboard, electronics, and wiring basics to build foundation for 5DOF robotic arm project. Started by wiring one led with resistor in series, then added button control, then added second LED and Joystick control. finally, used joystick and pulse width modulation to change brightness with joystick position. 

Key takeaways: 
* ADC converts voltage input to digital so that the rpi can interpret the signal. 
* LED brightness is controlled by rapidly turning LED on/off and modulating the delay between cycles. 
* Breadboards can have breaks in the middle of the power rail. 

## Part 2: Servo motor calibration

- Tested SG90 servos
- Found min/max pulse values and 0*/180* values
- Used serial input to manually find values

Learned: 
- One servo's value range was shifted up by 150-300
- Mechanical servo limits are different from true 0*/180* limits
- Will need external power to run all 6 servos for this projects due to pico power limits


## Part 3 Add control for all servos 

- Completed assembly of 5DOF arm 
- Manual control for all motors via serial input
- Slow movement function to control momentum of the arm
- Upgraded power to 6V external power (4x AA batteries)
- Removed gripper due to to torque limitations of SG90 servos

Learned: 

- SG90 servos have insufficient torque to be useful in this application. I'll need to design my own version of this arm for practical usage. 
- 3.7V power was insufficent to run all 6 servos. Had brown out the first time I ran it, but no problems after switching to 6V power. 

## Part 4 Implimenet 2D inverse kinematics

- Short script to test motors working simultaneously
- Inverse kinematics script to move actuator tip to a point in 2D space
- Used previous serial input control script to account for servo motor mounting angles
- Simplified arms into two links to make inital programming easier

Learned: 

- Running multiple servos simultaneously causes power brownout (will add a second battery holder)
- Script in current state calculates negative motor angles, causing failure to get to some points in the positive X axis

## Part 4 continued, Debug 2D inverse kinematics

- Full 2D IK working
- Cleaned up code
- Implimented dual configuration solver to solve negative motor angle problem
- Coordinated mulit servo movements 
- Workspace limit validation
- Added second battery pack in parallel to solve power brownout

Learned: 

- Need workspace limits for servo and link length constraints
- Need to calculate multiple IK solutions for elbow up/down configurations
- Accounting for my specific servo mounting angles (each arm requires custom remapping)
- Need a conversion layer between IK output and physical servo input
- Diagnosing correct vs incorrect rejections (workspace limits vs bugs)

## Part 5 Add joystick control for 2D positions

- Wired analog joystick for real time position control
- Implemented relative/incremental joystick control by adding delta to current 2D IK position
- Added boundary sliding for a smoother user experience
- Used real time position serial output to assist with debugging
- Full 2D workspace now accessible through joystick control
- Still have erratic movements when elbow needs to change orientation to reach new position

Learned: 

- ADC init and reading, same as part 1 joystick brightness control
- Tested absolute vs relative control (joystick is not precise enough and control is difficult with aboslute control)
- Workspace geometry and hardware constraints when navigating 2D space (servo angles and link lengths make many positons unreachable)

## Part 6 Add second joystick for 3D control

- Wired second joystick for 3 axis control, x/z on 'side' joystick, Y axis on 'top' joystick
(as if joysticks were mounted on a cube)
- Remapped ADC inputs
- Fixed Y axis to use base rotation angular movement instead of Cartesian translation, preventing arm dip/rise when rotating
- simplified boundary sliding logic, although still seems buggy. 

Remaining issues:
- Jerky control partly due to hardware limits.
- Coordinates mapped incorrectly

Learned: 
- Pico only has 3 ADC inputs
- Changing Y in Cartesian space changes radial distance. Y needs to be applied as rotation.

