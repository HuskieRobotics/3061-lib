/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.lib.team3061.gyro;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.team3061.RobotConfig;
import frc.robot.Constants;

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
    this.angularVelocityXStatusSignal = this.gyro.getAngularVelocityX();
    this.angularVelocityXStatusSignal.setUpdateFrequency(100);
    this.angularVelocityYStatusSignal = this.gyro.getAngularVelocityY();
    this.angularVelocityYStatusSignal.setUpdateFrequency(100);
    this.angularVelocityZStatusSignal = this.gyro.getAngularVelocityZ();
    this.angularVelocityZStatusSignal.setUpdateFrequency(100);

    if (Constants.getMode() == Constants.Mode.SIM) {
      this.gyroSim = this.gyro.getSimState();
      this.gyroSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    } else {
      this.gyroSim = null;
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    if (RobotConfig.getInstance().getPhoenix6Licensed()) {
      // this is a licensed method
      BaseStatusSignal.waitForAll(
          LOOP_PERIOD_SECS,
          this.yawStatusSignal,
          this.pitchStatusSignal,
          this.rollStatusSignal,
          this.angularVelocityXStatusSignal,
          this.angularVelocityYStatusSignal,
          this.angularVelocityZStatusSignal);
    } else {
      this.yawStatusSignal.refresh();
      this.pitchStatusSignal.refresh();
      this.rollStatusSignal.refresh();
      this.angularVelocityXStatusSignal.refresh();
      this.angularVelocityYStatusSignal.refresh();
      this.angularVelocityZStatusSignal.refresh();
    }

    inputs.connected = (this.yawStatusSignal.getError() == StatusCode.OK);
    inputs.yawDeg = this.yawStatusSignal.getValue();
    inputs.pitchDeg = this.pitchStatusSignal.getValue();
    inputs.rollDeg = this.rollStatusSignal.getValue();
    inputs.rollDegPerSec = this.angularVelocityXStatusSignal.getValue();
    inputs.pitchDegPerSec = this.angularVelocityYStatusSignal.getValue();
    inputs.yawDegPerSec = this.angularVelocityZStatusSignal.getValue();
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
}
