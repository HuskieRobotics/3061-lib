# 3061-lib </br>

Huskie Robotics, FRC Team 3061's, starter project and library focused on a swerve-based drivetrain. Supports Swerve Drive Specialties (SDS) MK4/MK4i/MK4n/MK5n swerve modules using devices from Cross the Road Electronics (CTRE): 2 Falcon 500 / Kraken motors and a CTRE CANCoder, a CTRE Pigeon Gyro along with REV Robotics power distribution hub and pneumatics hub. While, due to the hardware abstraction layer, this code can be adapted to other motor controllers, encoders, and gyros as well as different swerve module designs, only CTRE devices and SDS swerve modules are supported "out of the box."

**Is 3061-lib a Good Fit for Your Team?**
----
* If you just want to get your new serve drive up and running, there are less complicated options. For example, if you are using all CTRE hardware, you should use their [Swerve Project Generator](https://pro.docs.ctr-electronics.com/en/latest/docs/tuner/tuner-swerve/index.html).
* While 3061-lib has the potential to support Rev motors and sensors, that requires some work on your part. You may find AdvantageKit's [Swerve Drive Projects](https://docs.advantagekit.org/getting-started/template-projects/spark-swerve-template) more helpful.
* If your team is interested in incorporating the following features, 3061-lib may be a good fit for your team.

**Is Your Team Using 3061-lib?**
----
* Due to the nature of 3061-lib, we only know who is using this code when we hear from you. If you are using 3061-lib, we would appreciate it if you would complete [this form](https://forms.gle/NCVDfZ41bY9gz25S8) and let us know more about you and how you use 3061-lib.
* Do you have feedback on 3061-lib? Please feel free to leave feedback when completing the form or [open a GitHub issue](https://github.com/HuskieRobotics/3061-lib/issues/new/choose).

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
* Multiple robots with different configurations. This is useful for supporting multiple physical robots with the same code base (e.g., a practice bot and a competition bot). It is also useful when testing an independent mechanism (use the PracticeBoard configuration) or when testing and tuning vision (use the VisionTestPlatform configuration).
* Logging and replay via [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/README.md). While 3061-lib leverages CTRE's SwerveDrivetrain class, which sits below the AdvantageKit hardware abstraction layer, it has an additional pose estimator that receives updates from the SwerveDrivetrain class while residing above the hardware abstraction layer. This enables replay and tuning of poses. Import the AdvantageScope.json layout file saved at the root of this project in AdvantageScope for a layout that demonstrates visualization of much that is logged.
* Vision subsystem supporting multiple Photonvision-based cameras to update pose estimation.
* Move-to-pose command that generates on-the-fly paths using a field model defined by a set of regions and transition points between regions.
* CAN FD (CANivore) to reduce CAN bus utilization
* Monitoring and reporting of hardware faults
* Integrated system tests
* SysId routines, which can be selected with the SysID Chooser and executed with the specified button combinations (refer to FullOperatorConsoleOI.java).
* Commands
    * drive-to-pose (closed-loop straight-line motion to pose)
    * rotate-to-angle (closed-loop rotational setpoint with optional driver-controlled translational motion)
* Swerve-specific features
    * robot-relative and field-relative driving modes
    * slow mode for fine-tuned motion
    * current limiting configuration for motors
    * x-stance
    * switch steer and drive motors to coast mode when robot is disabled and has stopped moving for a period of time to facilitate manual alignment

**Configuration**
----
Each robot's configuration is captured in that robot's subclass of the ```RobotConfig``` abstract class. This enables the same code base to support multiple robots with different characteristics. Each enumerated value in the ```RobotType``` enumeration in Constants.java has a corresponding subclass of ```RobotConfig``` (other than ```ROBOT_SIMBOT```). For example the ```RobotType.ROBOT_DEFAULT``` corresponds to the ```DefaultRobotConfig``` subclass.

To configure the initial robot, address each of the ```FIXME``` comments in the ```DefaultRobotConfig``` class.

To add an additional robot, create a new subclass of ```RobotConfig``` (you can duplicate and rename ```DefaultRobotConfig```); add the new robot to the ```RobotType``` enumeration, update the ```getRobot``` and ```getMode``` methods, and update the ```ROBOT``` constant in Constants.java; and update the RobotContainer constructor to create an instance of the new RobotConfig subclass based on the value returned from ```Constants.getRobot()```.

**Tuning**
----

* If you are using all CTRE hardware, use the Swerve Project Generator in Tuner X. Copy the values from the generated TunerConstants.java file into your RobotConfig subclass, DrivetrainIOCTRE.java, and SwerveConstants.java as appropriate.
* If you are not using all CTRE hardware, do the following.
    * Setting Steer Offsets (e.g., ```FRONT_LEFT_MODULE_STEER_OFFSET_ROT```) in your ```RobotConfig``` subclass (e.g., DefaultRobotConfig.java):
        * For finding the offsets, use a piece of 2x1 extrusion that is straight against the forks of the front and back modules (on the left and right side) to ensure that the modules are straight
        * Point the bevel gears of all the wheels towards the center of the robot for MK3/4 and towards the outside of the robot for MK5.
        * Open Phoenix Tuner X and, for each CANcoder on a swerve module, copy the negated rotation value of the "Absolute Position No Offset" signal to the ```STEER_OFFSET``` constants. For example, if the CANcoder "Absolute Position No Offset" signal is 0.104004, specify a value of -0.104004 for the corresponding constant.
* Checking geometry:
    1. elevate the robot on a cart positioned in front of the operator console so the driver's perspective is the same as the robot's (i.e., the front of the robot facing away from the driver); we label the front, back, left, and right of the robot on the robot since which is which may be ambiguous until more of the robot is assembled
    2. position all four wheels such that they are pointing forward (bevel gears all facing the same direction as when the steer offsets were determined, which is towards the center of the robot)
    3. push the drive joystick forward
    4. verify that the joystick is specifying a positive x velocity (graph /RealOutput/TeleopSwerve/xVelocity in AdvantageScope); if not, update the corresponding method in the OperatorInterface subclass
    5. verify that each wheel is rotating to move the robot forward (+x direction)
    6. verify that the drive TalonFX distance is consistent (graph FL/OdometryDrivePositionMeters and verify that it is increasing; check all 4 modules)
    7. perform steps 3-6 but pull the drive joystick backwards (xVelocity should be negative and OdometryDrivePositionMeters should decrease)
    8. perform steps 3-6 but push the drive joystick to the left (yVelocity should be positive; OdometryDrivePositionMeters should increase if the wheel is pointed to the left and decrease if pointed to the right; add Drivetrain/SwerveModuleStates to the Swerve tab in AdvantageScope to visualize)
    9. perform steps 3-6 but push the drive joystick to the right (yVelocity should be negative; OdometryDrivePositionMeters should increase if the wheel is pointed to the right and decrease if pointed to the left; use the Swerve tab in AdvantageScope to visualize)
    10. push the rotate joystick in the direction to rotate CCW (when looking down on the robot)
    11. verify that the joystick is specifying a positive rotational velocity (graph /RealOutput/TeleopSwerve/rotationalVelocity in AdvantageScope); if not, update the corresponding method in the OperatorInterface subclass
    12. verify that each wheel is rotating to rotate the robot in a CCW direction
    13. perform steps 10-12 but push the rotate joystick in the direction to rotate CW
    14. graph /Drivetrain/RawHeadingDeg
    15. rotate the cart in a CCW direction and verify that the gyro is increasing
    16. rotate the cart in a CW direction and verify that the gyro is decreasing
    17. use Phoenix Tuner X to flash the LEDs on the TalonFX devices and CANcoders to ensure that the CAN IDs are properly assigned to the front-left, front-right, back-left, and back-right positions (all of the above may behave as expected even if the modules aren't assigned to the appropriate corners)
* For details on characterizing the steer and drive mechanism and tuning the associated PID, refer to [this document](https://docs.google.com/document/d/1maKiFmen-x5hGpW-QnDeSU37W0z5LyYb0b8MY0qDd-E/preview).
* ```AUTO_DRIVE_P_CONTROLLER``` and ```AUTO_TURN_P_CONTROLLER``` constants in in your ```RobotConfig``` subclass (e.g., DefaultRobotConfig.java):
    * set ```TUNING_MODE``` in Constants.java to true
    * open AdvantageScope, import the AdvantageScopeTuning.json layout file, and enable tuning; each of the PID values are under "/Tuning"
    * tune until until auto paths (e.g., Oval Test Fast) are smoothly followed
    * refer to the auto path tuning tab in Advantage Scope to minimize the errors
    * copy the values from AdvantageScope into DrivetrainConstants.java
    * set ```TUNING_MODE``` in Constants.java to false

**Joystick Mappings**
----
* This code is setup to use 2 joysticks to control the robot.
* Joystick 0 controls translation (forwards and sideways movement), and Joystick 1 controls rotation.
* Joystick 0's button 9 enables and disables field-relative driving.
* Joystick 0's button 4 zeroes the gyro, useful when testing teleop, just rotate the robot forwards, and press the button to rezero.
* Joystick 1's button 4 enables x-stance while pressed.

**Videos**
----
* [3061-lib Detailed Walkthrough Playlist](https://www.youtube.com/playlist?list=PLf5b4wwITZuy8Nx9a5lknND6lI_reLA3x)

**Additional References**
----
* [AdvantageScope Network Tables](https://docs.google.com/document/d/e/2PACX-1vRfNyTbYLsrKsvsglYrfjzFeAX9NwzxujVFkiw6YNhY4q8llXNcvPmTmMpet0ZBr3SQXdCxkkm_XV_E/pub): describes how we configure AdvantageScope and many of the useful values that are logged and where to view them in the provided AdvantageScope layout.
* [Coding a Subsystem](https://docs.google.com/document/d/e/2PACX-1vSoKDfaqDuGCELO67Z_OU1Wi1htoCOmoF5-17C2HNv1dCKMAVaq9HAGa5_nQpZRHAQsEgaKbTEiwgO4/pub): a more detailed description of what is required to create a new subsytem.
* [Command Scheduler](https://docs.google.com/document/d/e/2PACX-1vS5TPIvj47OJelcBR2K0lNPXy0A72G__a6bLnf2l0SGfUjM-TADPxHw7-s6GHQu5gSJ95nnCRfcgxQs/pub): flowchart of WPILib's Command Scheduler.
* [Competition Best Practices](https://docs.google.com/document/d/e/2PACX-1vQLStnC43iz8e7evLbORy99PDJclmt1k7UA60Lp2NWchMA0MyDMwknH0NvJk3FGo0KaARN6tOJ5sEnj/pub): best practices for reliability and traceability at competitions by leveraging the Event Deploy VS Code extension.
* [Coordinate Systems](https://docs.google.com/document/d/e/2PACX-1vRWPD7hRjByRldjwQ2oxGno31mvY6BRdi-WEIVUMj7v8jjKbIr_kbUJfd8_5l8kuwlUIjM39yFLAKMh/pub): an overview, with diagrams, of robot, field, Pigeon, and joystick coordinate systems.
* [Current Limiting](https://docs.google.com/document/d/e/2PACX-1vRYSPdktpdxEqfEJ5qOyJI1QZgMc7kVsm6Ehq9maGxWGrJPlZ-flu7b20wwI2OQtpVEouoiGKB02wDO/pub): description, rationale, and best practices for specifying current limits.
* [Operator Interface](https://docs.google.com/document/d/e/2PACX-1vSW9bMBWT1qbvids2dnbw3Nyx9KVRfCczwkKzmkih6P6htDaCBE4KAIzxbC2n-0mct3-kXr1Bc9JJqJ/pub): rationale and description for the Operator Interface design.
* [Phoenix 6 API Notes](https://docs.google.com/document/d/e/2PACX-1vQDTp-y6goSe_uPXCzGBasoqSanwskiwVedEaV50iE3vT0mL6Fs3bk0nt5iGK_R8BZ3371JZytDloSk/pub): most useful when migrating from Phoenix 5 to 6.
* [Raspberry Pi & PhotonVision Settings](https://docs.google.com/document/d/e/2PACX-1vRCiyodcgBIta9bTmd4BgXOtCwyXbhBM1mI0SPSmPaC4V4HNVHCPMO7Hqh1-xQTKebPE9WGIP0F3JQV/pub): how we configure PhotonVision for AprilTag detection
* [Robot Config](https://docs.google.com/document/d/e/2PACX-1vT_0-owsrGBXkkQI5R9a2-_YgukYuo5r8NEZFRvfdX9JiLqey7c9ew_8JmLjU84d7cHWUDx3FwtHRMc/pub): rationale and description for the Robot Config design.
* [State Machines](https://docs.google.com/document/d/e/2PACX-1vRWPD7hRjByRldjwQ2oxGno31mvY6BRdi-WEIVUMj7v8jjKbIr_kbUJfd8_5l8kuwlUIjM39yFLAKMh/pub): rationale, example diagrams, and best practices for state machines and the state machine design in 3061-lib.
* [Subsystem Testing and Tuning](https://docs.google.com/document/d/e/2PACX-1vT7eesqIg4E16QXMnDbtrsDJ6jEBApWDYRyNXWFtIHHf6upWCrLpBTFEyG6xL03H0mBDx3EbL_P5vCx/pub): how to perform initial tests on mechanisms, debug with log files, and tune constants.
* [Tuning Odometry](https://docs.google.com/document/d/e/2PACX-1vTCUeWsEIDd0dPoKiUDXsxO2hNxoeo5t55umNfDzaXLCjZHv5IoZo675Dm3F2JHVU9cVyAeMagXN3Hj/pub): rationale and good, better, best approaches to improving the accuracy of the robot's odometry.
* [Tuning Drivetrain and Auto](https://docs.google.com/document/d/e/2PACX-1vSWmUUMqQuSlFtFWMCQ1ERwa_Xk1kULSdQkEioN4UbdIryG4_MpbVKU6gZtM85ZTYz6-JKXBN9UsG25/pub): rationale and instructions on how to use SysId to characterize the drivetrain, tune PIDs for swerve, and tune PIDs for PathPlanner.


**Credits**
----
* MK4/MK4i code initially from Team 364's [BaseFalconSwerve](https://github.com/Team364/BaseFalconSwerve)
* general AdvantageKit logging code, AdvantageKit-enabled Gyro classes, swerve module simulation, and drive characterization initially from Mechanical Advantage's [SwerveDevelopment](https://github.com/Mechanical-Advantage/SwerveDevelopment)
* AdvantageKit-enabled pneumatics classes initially from Mechanical Advantage's 2022 [robot code](https://github.com/Mechanical-Advantage/RobotCode2022)
* AdvantageKit-enabled framework code and LED class initially from Mechanical Advantage's 2023 [robot code](https://github.com/Mechanical-Advantage/RobotCode2023)
* FaultReporter (originally AdvancedSubsystem), SubsystemFault, SelfChecking classes initially from [Ranger Robotics](https://github.com/3015RangerRobotics/2023Public)
