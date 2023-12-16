/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.lib.team3061.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;

public class GyroIOPigeon2Phoenix6 implements GyroIO {
  private final Pigeon2 gyro;
  private final StatusSignal<Double> yawStatusSignal;
  private final StatusSignal<Double> pitchStatusSignal;
  private final StatusSignal<Double> rollStatusSignal;
  private final StatusSignal<Double> angularVelocityXStatusSignal;
  private final StatusSignal<Double> angularVelocityYStatusSignal;
  private final StatusSignal<Double> angularVelocityZStatusSignal;
  private final Pigeon2SimState gyroSim;

  public GyroIOPigeon2Phoenix6(int id) {
    gyro = new Pigeon2(id, RobotConfig.getInstance().getCANBusName());
    this.yawStatusSignal = this.gyro.getYaw();
    this.yawStatusSignal.setUpdateFrequency(100);
    this.pitchStatusSignal = this.gyro.getPitch();
    this.pitchStatusSignal.setUpdateFrequency(100);
    this.rollStatusSignal = this.gyro.getRoll();
    this.rollStatusSignal.setUpdateFrequency(100);
    this.angularVelocityXStatusSignal = this.gyro.getAngularVelocityXWorld();
    this.angularVelocityXStatusSignal.setUpdateFrequency(100);
    this.angularVelocityYStatusSignal = this.gyro.getAngularVelocityYWorld();
    this.angularVelocityYStatusSignal.setUpdateFrequency(100);
    this.angularVelocityZStatusSignal = this.gyro.getAngularVelocityZWorld();
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
      this.yawStatusSignal.refresh();
      this.angularVelocityZStatusSignal.refresh();
    }

    this.pitchStatusSignal.refresh();
    this.rollStatusSignal.refresh();
    this.angularVelocityXStatusSignal.refresh();
    this.angularVelocityYStatusSignal.refresh();

    inputs.connected = (this.yawStatusSignal.getStatus() == StatusCode.OK);
    inputs.yawDeg =
        BaseStatusSignal.getLatencyCompensatedValue(
            this.yawStatusSignal, this.angularVelocityZStatusSignal);
    inputs.pitchDeg =
        BaseStatusSignal.getLatencyCompensatedValue(
            this.pitchStatusSignal, this.angularVelocityYStatusSignal);
    inputs.rollDeg =
        BaseStatusSignal.getLatencyCompensatedValue(
            this.rollStatusSignal, this.angularVelocityXStatusSignal);
    inputs.rollDegPerSec = this.angularVelocityXStatusSignal.getValue();
    inputs.pitchDegPerSec = this.angularVelocityYStatusSignal.getValue();
    inputs.yawDegPerSec = this.angularVelocityZStatusSignal.getValue();
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
  public List<StatusSignal<Double>> getOdometryStatusSignals() {
    ArrayList<StatusSignal<Double>> signals = new ArrayList<>();
    signals.add(this.yawStatusSignal);
    signals.add(this.angularVelocityZStatusSignal);
    return signals;
  }
}
