// from Team 5712

package frc.lib.team3061.swerve_drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.units.measure.*;

/**
 * SysId-specific SwerveRequest to characterize the steer module characteristics of a swerve
 * drivetrain.
 */
public class SysIdSwerveSteerGains_Torque implements SwerveRequest {
  /** Torque current to apply to steer motors. */
  public double TorqueCurrentToApply = 0;

  /* Local reference to a coast request for the drive motors */
  private final CoastOut m_driveRequest = new CoastOut();
  /* Local reference to a torque current request for the steer motors */
  private final TorqueCurrentFOC m_steerRequest = new TorqueCurrentFOC(0);

  public StatusCode apply(
      SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
    for (int i = 0; i < modulesToApply.length; ++i) {
      modulesToApply[i].apply(m_driveRequest, m_steerRequest.withOutput(TorqueCurrentToApply));
    }
    return StatusCode.OK;
  }

  /**
   * Sets the torque current to apply to the drive wheels.
   *
   * @param current Torque current to apply
   * @return this request
   */
  public SysIdSwerveSteerGains_Torque withTorqueCurrent(double current) {
    TorqueCurrentToApply = current;
    return this;
  }
  /**
   * Sets the torque current to apply to the drive wheels.
   *
   * @param current Torque current to apply
   * @return this request
   */
  public SysIdSwerveSteerGains_Torque withTorqueCurrent(Current current) {
    TorqueCurrentToApply = current.in(Amps);
    return this;
  }
}
