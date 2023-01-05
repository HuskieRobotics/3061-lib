package frc.robot.subsystems.vision;

import java.io.IOException;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.vision.VisionIO;
import frc.robot.RobotOdometry;

public class Vision extends SubsystemBase {
    private VisionIO visionIO;
    private AprilTagFieldLayout layout;
    
    private double lastTimestamp;
    private SwerveDrivePoseEstimator poseEstimator;

    public Vision(VisionIO visionIO) {
        this.visionIO = visionIO;
        this.poseEstimator = RobotOdometry.getInstance().getPoseEstimator();

        try {
            layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
        } catch (IOException e) {
            //TODO: figure out the best way to handle this (it shouldn't ever happen)
        }
    }

    public double getLatestTimestamp() {
        return visionIO.getLatestTimestamp();
    }

    public PhotonPipelineResult getLatestResult() {
        return visionIO.getLatestResult();
    }

    @Override
    public void periodic() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
        } else {
            layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        }

        if (lastTimestamp < getLatestTimestamp()) {
            lastTimestamp = getLatestTimestamp();
            for (PhotonTrackedTarget target : getLatestResult().getTargets()) {
                if (isValidTarget(target)) {
                    Transform3d cameraToTarget = target.getBestCameraToTarget();
                    Pose3d tagPose = layout.getTagPose(target.getFiducialId()).get(); // is this the right way to do this?
                    Pose3d cameraPose = tagPose.transformBy(cameraToTarget.inverse());
                    Pose3d robotPose = cameraPose.transformBy(VisionConstants.ROBOT_TO_CAMERA.inverse());
                    poseEstimator.addVisionMeasurement(robotPose.toPose2d(), getLatestTimestamp()); //TODO: verify adding multiple measurements at once doesn't break anything.
                    
                    Logger.getInstance().recordOutput("Vision/TagPose", tagPose);
                    Logger.getInstance().recordOutput("Vision/CameraPose", cameraPose);
                    Logger.getInstance().recordOutput("Vision/RobotPose", robotPose.toPose2d());
                }
            }
        }
    }
    

    public boolean tagVisible(int id) {
        PhotonPipelineResult result = getLatestResult();
        for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getFiducialId() == id && isValidTarget(target)) {
                return true;
            }
        }
        return false;
    }

    /**
     * returns the best Rotation3d from the robot to the given target. 
     * @param id
     * @return the transfomr3d or null if there isn't
     */

    public Transform3d getTransform3dToTag(int id) { 
        PhotonPipelineResult result = getLatestResult();
        for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getFiducialId() == id && isValidTarget(target)) {
                return VisionConstants.ROBOT_TO_CAMERA.plus(target.getBestCameraToTarget());
            }
        }
        return null;
    }

    public Rotation2d getAngleToTag(int id) { 
        Transform3d transform = getTransform3dToTag(id);
        if (transform != null) {
            return new Rotation2d(transform.getTranslation().getX(), transform.getTranslation().getY());
        } else {
            return null;
        }
    }

    public double getDistanceToTag(int id) {
        Transform3d transform = getTransform3dToTag(id);
        if (transform != null) {
            return transform.getTranslation().toTranslation2d().getNorm();
        } else {
            return -1;
        }
    }

    public static boolean isValidTarget(PhotonTrackedTarget target) {
        return target.getFiducialId() != -1 &&
            target.getPoseAmbiguity() != -1 && 
            target.getPoseAmbiguity() < VisionConstants.MAXIMUM_AMBIGUITY;
    }
}
