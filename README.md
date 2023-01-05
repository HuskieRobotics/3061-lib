# 3061-lib </br>

Huskie Robotics, FRC Team 3061's, starter project and library focused on a swerve-based drivetrain. Supports SDS MK4/MK4i swerve modules using 2 Falcon 500 motors and a CTRE CANCoder, a CTRE Pigeon Gyro, and REV Robotics power distribution hub and pneumatics hub. However, due to the hardware abstraction layer, this code can be adapted to other motor controllers, encoders, and gyros as well as different swerve module designs.

**Features**
----
* multiple robots, including a simulated robot with basic simulation of swerve modules
* logging and replay via [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/README.md)
* CAN FD (CANivore) to reduce CAN bus utilization
* reports devices missing from the CAN bus
* swerve-specific features
    * robot-relative and field-relative driving modes
    * current limiting configuration for motors
    * x-stance
    * leave wheels rotated in last direction when not driving to enable smooth continuation of motion
    * switch drive motors to coast mode when robot is disabled and has stopped moving to facilitate manual pushing

**Configuration**
----
The following constants must be adjusted in the Constants.java, DrivetrainConstants.java, and SwerveModuleConstants.java files (all distance units must be in meters, and rotation units in degrees):</br>

* Constants.java
    * ```CAN_BUS_NAME``` constant: set to the name of the CANivore CAN FD bus or leave as an empty string if not using CANivore
    * ```RobotType``` enumeration: update to reflect your robots; leave ```ROBOT_SIMBOT``` if using simulation; update the ```getRobot``` and ```getMode``` methods to reflect new robots
* DrivetrainConstants.java
    * swerve module motor controllers and encoders CAN ID constants (e.g., ```FRONT_LEFT_MODULE_DRIVE_MOTOR```, ```FRONT_LEFT_MODULE_STEER_MOTOR```, ```FRONT_LEFT_MODULE_STEER_ENCODER```): set to the assigned CAN IDs
    * steer offset constants (e.g., ```FRONT_LEFT_MODULE_STEER_OFFSET```): refer to the Tuning section
    * ```TRACKWIDTH_METERS``` constant: set to the center-to-center distance of left and right modules
    * ```WHEELBASE_METERS``` constant: set to the center-to-center distance of front and rear module wheels
    * ```ROBOT_WIDTH_WITH_BUMPERS``` constant: set to the width of the robot including its bumpers
    * ```ROBOT_LENGTH_WITH_BUMPERS``` constant: set to the length of the robot including its bumpers
    * ```MAX_VELOCITY_METERS_PER_SECOND``` and ```MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND``` constants: you can use the theoretical values, but it is better to physically drive the robot and find the actual max values.
    * ```PIGEON_ID``` constant: set to the CAN ID of the Pigeon; ensure that the gyro rotation is CCW+
* SwerveModuleConstants.java
    * ```WHEEL_DIAMETER_METERS``` constant: set to the diameter of the wheel in the swerve module
    * ```DRIVE_GEAR_RATIO``` and ```ANGLE_GEAR_RATIO``` constants: set to the gear ration of the swerve module's drive and turn mechanisms; the gear ratios must be defined such that they are greater than 1
    * ```DRIVE_MOTOR_INVERTED``` constant: when the drive motor is supplied a positive input, it turns the swerve module wheel such that the robot moves forward; if not, set to true
    * ```ANGLE_MOTOR_INVERTED``` constant: when the angle motor is supplied a positive input, it rotates the swerve module wheel such that the wheel rotates in the CCW direction; if not, set to true
    * ```CAN_CODER_INVERTED``` constant: when the angle motor rotates the swerve module wheel in a CCW direction, the CANcoder should increase its reading; if not, set to true


**Tuning**
----

* Setting Steer Offsets (e.g., ```FRONT_LEFT_MODULE_STEER_OFFSET```) in DrivetrainConstants.java
    * set ```DEBUGGING``` in SwerveModule.java to true
    * for finding the offsets, use a piece of 1x1 metal that is straight against the forks of the front and back modules (on the left and right side) to ensure that the modules are straight
    * point the bevel gears of all the wheels in the same direction (either facing left or right), and preferably you should have the wheels facing in the direction where a positive input to the drive motor drives forward; if for some reason you set the offsets with the wheels backwards, you can change the ```DRIVE_MOTOR_INVERTED``` to fix
    * open Shuffleboard, go to the SwerveModule tab, and see 4 indicators called "Mod 0 Cancoder", "Mod 1 Cancoder", etc. If you have already straightened the modules, copy those 4 numbers exactly (to 2 decimal places) to their respective ```STEER_OFFSET``` constants
    * set ```DEBUGGING``` in SwerveModule.java back to false
* Angle Motor PID Values (```ANGLE_KP```, ```ANGLE_KI```, ```ANGLE_KD```) in SwerveModuleConstants.java:
    * set ```TUNING_MODE``` in Constants.java to true
    * open Shuffleboard, go to the SmartDashboard tab, and see controls for each of the PID values; values can be changed via these controls as you interactively tune the controller
    * start with a low P value (0.01)
    * multiply by 10 until the module starts oscillating around the set point
    * scale back by searching for the value (for example, if it starts oscillating at a P of 10, then try (10 -> 5 -> 7.5 -> etc.)) until the module overshoots the setpoint but corrects with no oscillation
    * repeat the process for D; the D value will basically help prevent the overshoot
    * ignore I
    * copy the values from the Shuffleboard controls into SwerveModuleConstants.java
    * set ```TUNING_MODE``` in Constants.java to false
* Drive characterization values (```DRIVE_KS```, ```DRIVE_KV```, ```DRIVE_KA```) in SwerveModuleConstants.java:
    * in Shuffleboard, set the "Auto Routine" chooser to "Drive Characterization"
    * start the autonomous period
    * the ```FeedForwardCharacterization``` command will run and output the KS and KV values (you do not need to lock the modules straight forward as the code will keep them oriented in the forward direction)
    * copy the KS and KV values into SwerveModuleConstants.java
* Drive Motor PID Values (```DRIVE_KP```, ```DRIVE_KI```, ```DRIVE_KD```) in SwerveModuleConstants.java:
    * set ```TUNING_MODE``` in Constants.java to true
    * open Shuffleboard, go to the SmartDashboard tab, and see controls for each of the PID values; values can be changed via these controls as you interactively tune the controller
    * tune ```DRIVE_KP``` until it doesn't overshoot and doesn't oscillate around a target velocity
    * copy the values from the Shuffleboard controls into SwerveModuleConstants.java
    * set ```TUNING_MODE``` in Constants.java to false
* ```AUTO_DRIVE_P_CONTROLLER``` and ```AUTO_TURN_P_CONTROLLER``` constants in DrivetrainConstants.java:
    * set ```TUNING_MODE``` in Constants.java to true
    * open Shuffleboard, go to the SmartDashboard tab, and see controls for each of the PID values; values can be changed via these controls as you interactively tune the controller
    * tune until until auto paths are smoothly followed
    * copy the values from the Shuffleboard controls into DrivetrainConstants.java
    * set ```TUNING_MODE``` in Constants.java to false

**Joystick Mappings**
----
* This code is setup to use 2 joysticks to control the robot. </br>
* Joystick 0 controls translation (forwards and sideways movement), and Joystick 1 controls rotation. </br>
* Joystick 0's button 3 enables and disables field-relative driving.
* Joystick 1's button 3 zeroes the gyro, useful when testing teleop, just rotate the robot forwards, and press the button to rezero.
* Joystick 0's button 1 enables x-stance while pressed.

**Credits**
----
* MK4/MK4i code initially from Team 364's [BaseFalconSwerve](https://github.com/Team364/BaseFalconSwerve)
* general AdvantageKit logging code, AdvantageKit-enabled Gyro classes, swerve module simulation, and drive characterization from Mechanical Advantage's [SwerveDevelopment](https://github.com/Mechanical-Advantage/SwerveDevelopment)
* AdvantageKit-enabled pneumatics classes from Mechanical Advantage's 2022 [robot code](https://github.com/Mechanical-Advantage/RobotCode2022)
* Talon factories from Citrus Circuits 2022 [robot code](https://github.com/frc1678/C2022)
* CAN device finder code from team 3620 2020 [robot code](https://github.com/FRC3620/FRC3620_2020_GalacticSenate)
