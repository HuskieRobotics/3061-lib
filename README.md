# 3061-lib </br>

Huskie Robotics, FRC Team 3061's, starter project and library focused on a swerve-based drivetrain. Supports SDS MK4/MK4i swerve modules using 2 Falcon 500 motors and a CTRE CANCoder, a CTRE Pigeon Gyro, and REV Robotics power distribution hub and pneumatics hub. However, due to the hardware abstraction layer, this code can be adapted to other motor controllers, encoders, and gyros as well as different swerve module designs.

**External Dependencies**
----
* git must be installed as the GVersion plugin depends upon it

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
Each robot's configuration is captured in that robot's subclass of the ```RobotConfig``` abstract class. This enables the same code base to support multiple robots with different characteristics. Each enumerated value in the ```RobotType``` enumeration in Constants.java has a corresponding subclass of ```RobotConfig``` (other than ```ROBOT_SIMBOT```). For example the ```RobotType.ROBOT_DEFAULT``` corresponds to the ```DefaultRobotConfig``` subclass.

To configure the initial robot, address each of the ```FIXME``` comments in the ```DefaultRobotConfig``` class.

To add an additional robot, create a new subclass of ```RobotConfig``` (you can duplicate and rename ```DefaultRobotConfig```); add the new robot to the ```RobotType``` enumeration, update the ```getRobot``` and ```getMode``` methods, and update the ```ROBOT``` constant in Constants.java; and update the RobotContainer constructor to create an instance of the new RobotConfig subclass based on the value returned from ```Constants.getRobot()```.

**Tuning**
----

* Checking motor and encoder directions:
    * The drive motor, angle motor, and angle encoder constants in SwerveModuleConstants should be configured appropriately for the MK4 and MK4i swerve modules. However, it doesn't hurt to verify.
    * When the drive motor is supplied a positive input, it should turn the swerve module wheel such that the robot moves forward; if not, negate.
    * When the angle motor is supplied a positive input, it should rotate the swerve module wheel such that the wheel rotates in the CCW direction; if not, negate.
    * When the angle motor rotates the swerve module wheel in a CCW direction, the CANcoder should increase its reading; if not, set to negate.
* Checking geometry:
    1. elevate the robot on a cart positioned in front of the operator console so the driver's perspective is the same as the robot's (i.e., the front of the robot facing away from the driver); we label the front, back, left, and right of the robot on the robot since which is which may be ambiguous until more of the robot is assembled
    2. position all four wheels such that they are pointing forward (bevel gears all facing the same direction as when the steer offsets were determined)
    3. push the drive joystick forward
    4. verify that the joystick is specifying a positive x velocity (graph AdvantageKit/RealOutput/TeleopSwerve/xVelocity in AdvantageScope); if not, update the corresponding method in the OperatorInterface subclass
    5. verify that each wheel is rotating to move the robot forward (+x direction)
    6. verify that the drive TalonFX distance is consistent (graph AdvantageKit/Mod0/DriveDistanceMeters and verify that it is increasing; check all 4 modules)
    7. perform steps 3-6 but pull the drive joystick backwards (xVelocity should be negative and DriveDistanceMeters should decrease)
    8. perform steps 3-6 but push the drive joystick to the left (yVelocity should be positive; DriveDistanceMeters should increase if the wheel is pointed to the left and decrease if pointed to the right; add AdvantageKit/RealOutputs/SwerveModuleStates to the Swerve tab in AdvantageScope to visualize)
    9. perform steps 3-6 but push the drive joystick to the right (yVelocity should be negative; DriveDistanceMeters should increase if the wheel is pointed to the right and decrease if pointed to the left; use the Swerve tab in AdvantageScope to visualize)
    10. push the rotate joystick in the direction to rotate CCW (when looking down on the robot)
    11. verify that the joystick is specifying a positive rotational velocity (graph AdvantageKit/RealOutput/TeleopSwerve/rotationalVelocity in AdvantageScope); if not, update the corresponding method in the OperatorInterface subclass
    12. verify that each wheel is rotating to rotate the robot in a CCW direction
    13. perform steps 10-12 but push the rotate joystick in the direction to rotate CW
    14. graph AdvantageKit/Drive/Gyro/PositionDeg
    15. rotate the cart in a CCW direction and verify that the gyro is increasing
    16. rotate the cart in a CW direction and verify that the gyro is decreasing
    17. use Phoenix Tuner to flash the LEDs on the Falcon 500s and CANcoders to ensure that the CAN IDs are properly assigned to the front-left, front-right, back-left, and back-right positions (all of the above may behave as expected even if the modules aren't assigned to the appropriate corners)
* Setting Steer Offsets (e.g., ```FRONT_LEFT_MODULE_STEER_OFFSET```) in your ```RobotConfig``` subclass (e.g., DefaultRobotConfig.java):
    * set ```DEBUGGING``` in SwerveModule.java to true
    * for finding the offsets, use a piece of 1x1 metal that is straight against the forks of the front and back modules (on the left and right side) to ensure that the modules are straight
    * point the bevel gears of all the wheels in the same direction (either facing left or right), and preferably you should have the wheels facing in the direction where a positive input to the drive motor drives forward; if for some reason you set the offsets with the wheels backwards, you can change the appropriate ```DRIVE_MOTOR_INVERTED``` in ```SwerveModuleConstants``` to fix
    * open Shuffleboard, go to the SwerveModule tab, and see 4 indicators called "Mod 0 Cancoder", "Mod 1 Cancoder", etc. If you have already straightened the modules, copy those 4 numbers exactly (to 2 decimal places) to their respective ```STEER_OFFSET``` constants
    * set ```DEBUGGING``` in SwerveModule.java back to false
* Angle Motor PID Values (```ANGLE_KP```, ```ANGLE_KI```, ```ANGLE_KD```) in your ```RobotConfig``` subclass (e.g., DefaultRobotConfig.java):
    * set ```TUNING_MODE``` in Constants.java to true
    * open Shuffleboard, go to the SmartDashboard tab, and see controls for each of the PID values; values can be changed via these controls as you interactively tune the controller
    * start with a low P value (0.01)
    * multiply by 10 until the module starts oscillating around the set point
    * scale back by searching for the value (for example, if it starts oscillating at a P of 10, then try (10 -> 5 -> 7.5 -> etc.)) until the module overshoots the setpoint but corrects with no oscillation
    * repeat the process for D; the D value will basically help prevent the overshoot
    * ignore I
    * copy the values from the Shuffleboard controls into SwerveModuleConstants.java
    * set ```TUNING_MODE``` in Constants.java to false
* Drive characterization values (```DRIVE_KS```, ```DRIVE_KV```, ```DRIVE_KA```) in in your ```RobotConfig``` subclass (e.g., DefaultRobotConfig.java):
    * in Shuffleboard, set the "Auto Routine" chooser to "Drive Characterization"
    * start the autonomous period
    * the ```FeedForwardCharacterization``` command will run and output the KS and KV values (you do not need to lock the modules straight forward as the code will keep them oriented in the forward direction)
    * copy the KS and KV values into SwerveModuleConstants.java
* Drive Motor PID Values (```DRIVE_KP```, ```DRIVE_KI```, ```DRIVE_KD```) in your ```RobotConfig``` subclass (e.g., DefaultRobotConfig.java):
    * set ```TUNING_MODE``` in Constants.java to true
    * open Shuffleboard, go to the SmartDashboard tab, and see controls for each of the PID values; values can be changed via these controls as you interactively tune the controller
    * in Shuffleboard, set the "Auto Routine" chooser to "Drive Velocity Tuning"
    * tune ```DRIVE_KP``` until it doesn't overshoot and doesn't oscillate around a target velocity
    * copy the values from the Shuffleboard controls into SwerveModuleConstants.java
    * set ```TUNING_MODE``` in Constants.java to false
* ```AUTO_DRIVE_P_CONTROLLER``` and ```AUTO_TURN_P_CONTROLLER``` constants in in your ```RobotConfig``` subclass (e.g., DefaultRobotConfig.java):
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
