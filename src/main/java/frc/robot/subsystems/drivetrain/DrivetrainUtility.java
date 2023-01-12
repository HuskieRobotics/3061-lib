package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;



public class DrivetrainUtility{

    private static Translation2d mostRecentRequestedVelocity = new Translation2d();
    private static double mostRecentRequestedRotation = 0;
    private static double mostRecentLoopTime = 0;
    /**
     * 
     * @param desiredVelocity the desired restricted velocity
     * @param currentVSpeeds    the current velocity of the chassis
     * @param accelerationLimit  the limit for how fast the robot can accelerate. Prevents issues such as faceplanting
     * @param rotatingAccelerationLimit the limit for how fast the robot can turn
     * @param gyroAngle the current gyro angle
     * @return
     */
    public static Translation2d limitAcceleration(ChassisSpeeds desiredVelocity, ChassisSpeeds currentVSpeeds, Double accelerationLimit, Double rotatingAccelerationLimit, Rotation2d gyroAngle){
        ChassisSpeeds commandedVelocity;
        


        Translation2d velocity = ChassisSpeeds.fromFieldRelativeSpeeds(new Translation2d(
                        commandedVelocity.vxMetersPerSecond,
                        commandedVelocity.vyMetersPerSecond),
                gyroAngle);