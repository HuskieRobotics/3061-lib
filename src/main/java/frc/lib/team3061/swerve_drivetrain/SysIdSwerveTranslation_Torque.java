// from Team 5712

package frc.lib.team3061.swerve_drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.units.measure.*;

/**
 * SysId-specific SwerveRequest to characterize the translational characteristics of a swerve
 * drivetrain.
 */
public class SysIdSwerveTranslation_Torque implements SwerveRequest {
  /** Torque current to apply to drive wheels. */
  public double TorqueCurrentToApply = 0;

  /* Local reference to a torque current request for the drive motors */
  private final TorqueCurrentFOC m_driveRequest = new TorqueCurrentFOC(0);
  /* Local reference to a position request for the steer motors */
  // Select one of the two below based on which type of steer closed-loop you are using
  private final PositionVoltage m_steerRequest = new PositionVoltage(0);
  // private final PositionTorqueCurrentFOC m_steerRequest = new PositionTorqueCurrentFOC(0);

  public StatusCode apply(
      SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
    for (int i = 0; i < modulesToApply.length; ++i) {
      modulesToApply[i].apply(
          m_driveRequest.withOutput(TorqueCurrentToApply), m_steerRequest.withPosition(0));
    }
    return StatusCode.OK;
  }

  /**
   * Sets the torque current to apply to the drive wheels.
   *
   * @param current Torque current to apply
   * @return this request
   */
  public SysIdSwerveTranslation_Torque withTorqueCurrent(double current) {
    TorqueCurrentToApply = current;
    return this;
  }
  /**
   * Sets the torque current to apply to the drive wheels.
   *
   * @param current Torque current to apply
   * @return this request
   */
  public SysIdSwerveTranslation_Torque withTorqueCurrent(Current current) {
    TorqueCurrentToApply = current.in(Amps);
    return this;
  }
}
