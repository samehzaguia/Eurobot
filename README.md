This is the arduino code for the competition Eurobot in its 2020 Edition. 
The code mainly handles the navigation part and communicates with the following components: 
- Roboclaw Motobor Controller  https://www.basicmicro.com/RoboClaw-2x15A-Motor-Controller_p_10.html . This component uses input from encoders for position control using different algorithms( PI, PID, etc.)
- STM32 Discovery Board for mechanical functions such as pick and place operations. 

The robot uses the different weighpoints, deducted from the competition map. These weughpoints can be easily changed based on the game strategy to collect the maximum of score points. 
