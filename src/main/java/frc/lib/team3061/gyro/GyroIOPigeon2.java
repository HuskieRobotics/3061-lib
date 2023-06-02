/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.lib.team3061.gyro;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import frc.lib.team3061.RobotConfig;

public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 gyro;
  private final double[] xyzDps = new double[3];

  public GyroIOPigeon2(int id) {
    gyro = new Pigeon2(id, RobotConfig.getInstance().getCANBusName());
    this.gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 9);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    gyro.getRawGyro(xyzDps);
    inputs.connected = gyro.getLastError().equals(ErrorCode.OK);
    inputs.yawDeg = gyro.getYaw(); // degrees
    inputs.pitchDeg = gyro.getPitch(); // degrees
    inputs.rollDeg = gyro.getRoll(); // degrees
    inputs.rollDegPerSec = xyzDps[0]; // degrees per second
    inputs.pitchDegPerSec = xyzDps[1]; // degrees per second
    inputs.yawDegPerSec = xyzDps[2]; // degrees per second
  }
}
