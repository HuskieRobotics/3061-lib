/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.lib.team3061.gyro;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;

public class GyroIOPigeon2Phoenix6 implements GyroIO {
  private final Pigeon2 gyro;
  private final StatusSignal<Angle> yawStatusSignal;
  private final StatusSignal<Angle> pitchStatusSignal;
  private final StatusSignal<Angle> rollStatusSignal;
  private final StatusSignal<AngularVelocity> angularVelocityXStatusSignal;
  private final StatusSignal<AngularVelocity> angularVelocityYStatusSignal;
  private final StatusSignal<AngularVelocity> angularVelocityZStatusSignal;
  private final Pigeon2SimState gyroSim;

  public GyroIOPigeon2Phoenix6(int id) {
    gyro = new Pigeon2(id, RobotConfig.getInstance().getCANBusName());
    this.yawStatusSignal = this.gyro.getYaw().clone();
    this.yawStatusSignal.setUpdateFrequency(100);
    this.pitchStatusSignal = this.gyro.getPitch().clone();
    this.pitchStatusSignal.setUpdateFrequency(100);
    this.rollStatusSignal = this.gyro.getRoll().clone();
    this.rollStatusSignal.setUpdateFrequency(100);
    this.angularVelocityXStatusSignal = this.gyro.getAngularVelocityXWorld().clone();
    this.angularVelocityXStatusSignal.setUpdateFrequency(100);
    this.angularVelocityYStatusSignal = this.gyro.getAngularVelocityYWorld().clone();
    this.angularVelocityYStatusSignal.setUpdateFrequency(100);
    this.angularVelocityZStatusSignal = this.gyro.getAngularVelocityZWorld().clone();
    this.angularVelocityZStatusSignal.setUpdateFrequency(100);

    FaultReporter.getInstance().registerHardware("Drivetrain", "gyro", gyro);

    if (Constants.getMode() == Constants.Mode.SIM) {
      this.gyroSim = this.gyro.getSimState();
    } else {
      this.gyroSim = null;
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // only invoke refresh if Phoenix is not licensed (if licensed, these signals have already been
    // refreshed)
    if (!RobotConfig.getInstance().getPhoenix6Licensed()) {
      BaseStatusSignal.refreshAll(this.yawStatusSignal, this.angularVelocityZStatusSignal);
    } else {
      BaseStatusSignal.refreshAll(
          this.yawStatusSignal,
          this.angularVelocityZStatusSignal,
          this.pitchStatusSignal,
          this.rollStatusSignal,
          this.angularVelocityXStatusSignal,
          this.angularVelocityYStatusSignal);
    }

    inputs.connected = (this.yawStatusSignal.getStatus() == StatusCode.OK);
    inputs.yawDeg =
        BaseStatusSignal.getLatencyCompensatedValue(
                this.yawStatusSignal, this.angularVelocityZStatusSignal)
            .in(Degrees);
    inputs.pitchDeg =
        BaseStatusSignal.getLatencyCompensatedValue(
                this.pitchStatusSignal, this.angularVelocityYStatusSignal)
            .in(Degrees);
    inputs.rollDeg =
        BaseStatusSignal.getLatencyCompensatedValue(
                this.rollStatusSignal, this.angularVelocityXStatusSignal)
            .in(Degrees);
    inputs.rollDegPerSec = this.angularVelocityXStatusSignal.getValue().in(DegreesPerSecond);
    inputs.pitchDegPerSec = this.angularVelocityYStatusSignal.getValue().in(DegreesPerSecond);
    inputs.yawDegPerSec = this.angularVelocityZStatusSignal.getValue().in(DegreesPerSecond);

    inputs.odometryYawPositions = new Rotation2d[] {Rotation2d.fromDegrees(inputs.yawDeg)};

    if (Constants.getMode() == Constants.Mode.SIM) {
      this.gyroSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    }
  }

  @Override
  public void setYaw(double yaw) {
    this.gyro.setYaw(yaw, 0.1);
  }

  @Override
  public void addYaw(double yaw) {
    if (Constants.getMode() == Constants.Mode.SIM) {
      this.gyroSim.addYaw(yaw);
    }
  }

  @Override
  public List<BaseStatusSignal> getOdometryStatusSignals() {
    ArrayList<BaseStatusSignal> signals = new ArrayList<>();
    signals.add(this.yawStatusSignal);
    signals.add(this.angularVelocityZStatusSignal);
    return signals;
  }
}
