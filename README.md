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

