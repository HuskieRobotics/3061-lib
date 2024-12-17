# 3061-lib </br>

Huskie Robotics, FRC Team 3061's, starter project and library focused on a swerve-based drivetrain. Supports SDS MK4/MK4i swerve modules using 2 Falcon 500 / Kraken motors and a CTRE CANCoder, a CTRE Pigeon Gyro, and REV Robotics power distribution hub and pneumatics hub. However, due to the hardware abstraction layer, this code can be adapted to other motor controllers, encoders, and gyros as well as different swerve module designs.

**Is 3061-lib a Good Fit for Your Team?**
----
* If you just want to get your new serve drive up and running, there are less complicated options. For example, if you are using all Cross The Road Electronics hardware, you should use their [Swerve Project Generator](https://pro.docs.ctr-electronics.com/en/latest/docs/tuner/tuner-swerve/index.html).
* While 3061-lib has the potential to support Rev motors and sensors, that requires some work on your part. You may find AdvantageKit's [Swerve Drive Projects](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/INSTALLATION.md#new-projects) more helpful.
* If your team is interested in incorporating the following features, 3061-lib may be a good fit for your team.

**External Dependencies**
----
* git must be installed as the GVersion plugin depends upon it
* [CTRE's Phoenix 6](https://store.ctr-electronics.com/software/) (both free and licensed options are supported)
* [RevLib](https://docs.revrobotics.com/sparkmax/software-resources/spark-max-api-information)
* [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/README.md)
* [PathPlanner](https://github.com/mjansen4857/pathplanner)
* [PhotonLib](https://photonvision.org)

**Features**
----
* multiple robots with different configurations, including a simulated robot with basic simulation of swerve modules
* logging and replay via [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/README.md)
* vision subsystem supporting multiple Photonvision-based cameras to update pose estimation
* move-to-pose command that generates on-the-fly paths using a field model defined by a set of regions and transition points between regions
* CAN FD (CANivore) to reduce CAN bus utilization
* monitoring and reporting of hardware faults
* integrated system tests
* commands
    * drive-to-pose (closed-loop straight-line motion to pose)
    * rotate-to-angle (closed-loop rotational setpoint with optional driver-controlled translational motion)
* swerve-specific features
    * robot-relative and field-relative driving modes
    * slow mode for fine-tuned motion
    * acceleration limiting when not in "turbo" mode
    * current limiting configuration for motors
    * x-stance
    * switch drive motors to coast mode when robot is disabled and has stopped moving for a period of time to facilitate manual pushing

**Configuration**
----
Each robot's configuration is captured in that robot's subclass of the ```RobotConfig``` abstract class. This enables the same code base to support multiple robots with different characteristics. Each enumerated value in the ```RobotType``` enumeration in Constants.java has a corresponding subclass of ```RobotConfig``` (other than ```ROBOT_SIMBOT```). For example the ```RobotType.ROBOT_DEFAULT``` corresponds to the ```DefaultRobotConfig``` subclass.

To configure the initial robot, address each of the ```FIXME``` comments in the ```DefaultRobotConfig``` class.

To add an additional robot, create a new subclass of ```RobotConfig``` (you can duplicate and rename ```DefaultRobotConfig```); add the new robot to the ```RobotType``` enumeration, update the ```getRobot``` and ```getMode``` methods, and update the ```ROBOT``` constant in Constants.java; and update the RobotContainer constructor to create an instance of the new RobotConfig subclass based on the value returned from ```Constants.getRobot()```.

**Tuning**
----

* If you are using all CTRE hardware, use the Swerve Project Generator in Tuner X. Copy the values from the generated TunerConstants.java file into your RobotConfig subclass, DrivetrainIOCTRE.java, and SwerveConstants.java as appropriate.
* If you are not using all CTRE hardware, do the following.
    * Checking motor and encoder directions:
        * The drive motor, angle motor, and angle encoder constants in SwerveModuleConstants should be configured appropriately for the MK4 and MK4i swerve modules. However, it doesn't hurt to verify.
        * If you want to verify the following, the ```Subsystem``` example subsystem can be used with ```TESTING``` set to true to directly control the associated motor. Set ```MOTOR_CAN_ID``` in ```SubsystemConstants``` to the appropriate CAN ID and ensure that CAN ID is not specified for any swerve motor in ```DefaultRobotConfig```. Use Shuffleboard to set the motor power.
        * When the drive motor is supplied a positive input, it should turn the swerve module wheel such that the robot moves forward; if not, negate.
        * When the angle motor is supplied a positive input, it should rotate the swerve module wheel such that the wheel rotates in the CCW direction; if not, negate.
        * When the angle motor rotates the swerve module wheel in a CCW direction, the CANcoder should increase its reading; if not, set to negate.
    * Setting Steer Offsets (e.g., ```FRONT_LEFT_MODULE_STEER_OFFSET_ROT```) in your ```RobotConfig``` subclass (e.g., DefaultRobotConfig.java):
        * For finding the offsets, use a piece of 2x1 extrusion that is straight against the forks of the front and back modules (on the left and right side) to ensure that the modules are straight
        * Point the bevel gears of all the wheels in the same direction (either facing left or right), and preferably you should have the wheels facing in the direction where a positive input to the drive motor drives forward; if for some reason you set the offsets with the wheels backwards, you can change the appropriate ```DRIVE_MOTOR_INVERTED``` in ```SwerveModuleConstants``` to fix
        * Open Phoenix Tuner X and, for each CANcoder on a swerve module, copy the negated rotation value of the "Absolute Position No Offset" signal to the ```STEER_OFFSET``` constants. For example, if the CANcoder "Absolute Position No Offset" signal is 0.104004, specify a value of -0.104004 for the corresponding constant.
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
    17. use Phoenix Tuner X to flash the LEDs on the Falcon 500s and CANcoders to ensure that the CAN IDs are properly assigned to the front-left, front-right, back-left, and back-right positions (all of the above may behave as expected even if the modules aren't assigned to the appropriate corners)
* If you are using voltage control outputs:
    * Angle characterization values (```ANGLE_KS```, ```ANGLE_KV```, ```ANGLE_KA```) in in your ```RobotConfig``` subclass (e.g., DefaultRobotConfig.java):
        * set ```TUNING_MODE``` in Constants.java to true (characterization analysis doesn't normally run)
        * in Shuffleboard, set the "Auto Routine" chooser to "Swerve Rotate Characterization"
        * start the autonomous period
        * the ```FeedForwardCharacterization``` command will run and output the KS, KV, and KA values
        * copy the KS, KV, and KA values into the corresponding constants in your robot config file
    * Drive characterization values (```DRIVE_KS```, ```DRIVE_KV```, ```DRIVE_KA```) in in your ```RobotConfig``` subclass (e.g., DefaultRobotConfig.java):
        * set ```TUNING_MODE``` in Constants.java to true (characterization analysis doesn't normally run)
        * in Shuffleboard, set the "Auto Routine" chooser to "Swerve Drive Characterization"
        * ensure that there is about 40' of carpet in front of the robot before enabling the auto routine; if there is less, increase ```RAMP_RATE_VOLTS_PER_SECOND``` in ```FeedForwardCharacterization```
        * start the autonomous period
        * the ```FeedForwardCharacterization``` command will run and output the KS, KV, KA values (you do not need to lock the modules straight forward as the code will keep them oriented in the forward direction)
        * copy the KS, KV, and KA values into the corresponding constants in your robot config file
* If you are using TorqueCurrentFOC (Phoenix Pro) control outputs:
    * Angle characterization values (```ANGLE_KS```, ```ANGLE_KV```, ```ANGLE_KA```) in in your ```RobotConfig``` subclass (e.g., DefaultRobotConfig.java):
        * set the robot on the carpet
        * set ```ANGLE_KV``` to 0 since current corresponds to acceleration, not velocity
        * set ```TESTING``` in DrivetrainConstants.java to true
        * using Shuffleboard or AdvantageScope, increase the value of the Drivetrain/steerCurrent value until the swerve modules start to rotate; set ```ANGLE_KS``` to this value
        * using AdvantageScope, create a Line Graph and plot the acceleration of each steer motor
        * set the value of Drivetrain/steerCurrent to 10 A
        * inspect the portion of the graph where the acceleration is constant (before the motor reached free speed); set ```ANGLE_KA``` to 10 minus ```ANGLE_KS``` divided by the average of the values from each steer motor
        * set ```TESTING``` in DrivetrainConstants.java back to false
        * verify kA by using VelocityTorqueCurrentFOC in Tuner X and commanding an acceleration (and a velocity if you want kS in circuit) (while you're far enough from free speed, the acceleration signal should closely match the requested acceleration)
    * Drive characterization values (```DRIVE_KS```, ```DRIVE_KV```, ```DRIVE_KA```) in in your ```RobotConfig``` subclass (e.g., DefaultRobotConfig.java):
        * set the robot on the carpet
        * set ```DRIVE_KV``` to 0 since current corresponds to acceleration, not velocity
        * set ```TESTING``` in DrivetrainConstants.java to true
        * using Shuffleboard or AdvantageScope, increase the value of the Drivetrain/driveCurrent value until the wheels start to rotate; set ```DRIVE_KS``` to this value
        * using AdvantageScope, create a Line Graph and plot the acceleration of each drive motor
        * set the value of Drivetrain/driveCurrent to 40 A
        * inspect the portion of the graph where the acceleration is constant (before the motor reached free speed); set ```DRIVE_KA``` to 40 minus ```DRIVE_KS``` divided by the average of the values from each drive motor
        * set ```TESTING``` in DrivetrainConstants.java back to false
        * verify kA by using VelocityTorqueCurrentFOC in Tuner X and commanding an acceleration (and a velocity if you want kS in circuit) (while you're far enough from free speed, the acceleration signal should closely match the requested acceleration)
* Angle Motor PID Values (```ANGLE_KP```, ```ANGLE_KI```, ```ANGLE_KD```) in your ```RobotConfig``` subclass (e.g., DefaultRobotConfig.java):
    * set ```TUNING_MODE``` in Constants.java to true
    * open Shuffleboard, go to the SmartDashboard tab, and see controls for each of the PID values; values can be changed via these controls as you interactively tune the controller
    * start with a P value of 100.0
    * double until the module starts oscillating around the set point
    * scale back by searching for the value (for example, if it starts oscillating at a P of 100, then try (100 -> 50 -> 75 -> etc.)) until the module overshoots the setpoint but corrects with no oscillation
    * graph the ```anglePositionErrorDeg``` in Advantage Scope for each swerve module to assist in tuning
    * repeat the process for D; the D value will basically help prevent the overshoot
    * ignore I
    * copy the values from the Shuffleboard controls into SwerveModuleConstants.java
    * set ```TUNING_MODE``` in Constants.java to false
* Drive Motor PID Values (```DRIVE_KP```, ```DRIVE_KI```, ```DRIVE_KD```) in your ```RobotConfig``` subclass (e.g., DefaultRobotConfig.java):
    * set ```TUNING_MODE``` in Constants.java to true
    * open Shuffleboard, go to the SmartDashboard tab, and see controls for each of the PID values; values can be changed via these controls as you interactively tune the controller
    * in Shuffleboard, set the "Auto Routine" chooser to "Drive Velocity Tuning"
    * tune ```DRIVE_KP``` until it doesn't overshoot and doesn't oscillate around a target velocity
        * start with a P value of 0.2 (DrivetrainIOCTRE) or 0.005 (DrivetrainIOGeneric)
    * graph the ```driveVelocityErrorMetersPerSec``` in Advantage Scope for each swerve module to assist in tuning
    * copy the values from the Shuffleboard controls into SwerveModuleConstants.java
    * set ```TUNING_MODE``` in Constants.java to false
* ```AUTO_DRIVE_P_CONTROLLER``` and ```AUTO_TURN_P_CONTROLLER``` constants in in your ```RobotConfig``` subclass (e.g., DefaultRobotConfig.java):
    * set ```TUNING_MODE``` in Constants.java to true
    * open Shuffleboard, go to the SmartDashboard tab, and see controls for each of the PID values; values can be changed via these controls as you interactively tune the controller
    * tune until until auto paths are smoothly followed
    * graph the ```PathFollowing/xPosError```, ```PathFollowing/yPosError```, and ```PathFollowing/rotationError``` in Advantage Scope for each swerve module to assist in tuning
    * copy the values from the Shuffleboard controls into DrivetrainConstants.java
    * set ```TUNING_MODE``` in Constants.java to false

**Joystick Mappings**
----
* This code is setup to use 2 joysticks to control the robot.
* Joystick 0 controls translation (forwards and sideways movement), and Joystick 1 controls rotation.
* Joystick 0's button 9 enables and disables field-relative driving.
* Joystick 0's button 4 zeroes the gyro, useful when testing teleop, just rotate the robot forwards, and press the button to rezero.
* Joystick 1's button 4 enables x-stance while pressed.

**Credits**
----
* MK4/MK4i code initially from Team 364's [BaseFalconSwerve](https://github.com/Team364/BaseFalconSwerve)
* general AdvantageKit logging code, AdvantageKit-enabled Gyro classes, swerve module simulation, and drive characterization initially from Mechanical Advantage's [SwerveDevelopment](https://github.com/Mechanical-Advantage/SwerveDevelopment)
* AdvantageKit-enabled pneumatics classes initially from Mechanical Advantage's 2022 [robot code](https://github.com/Mechanical-Advantage/RobotCode2022)
* AdvantageKit-enabled framework code and LED class initially from Mechanical Advantage's 2023 [robot code](https://github.com/Mechanical-Advantage/RobotCode2023)
* FaultReporter (originally AdvancedSubsystem), SubsystemFault, SelfChecking classes initially from [Ranger Robotics](https://github.com/3015RangerRobotics/2023Public)
