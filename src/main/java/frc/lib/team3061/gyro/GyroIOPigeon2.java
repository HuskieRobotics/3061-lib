/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.lib.team3061.gyro;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import edu.wpi.first.math.util.Units;
import frc.lib.team3061.RobotConfig;
import org.littletonrobotics.junction.Logger;

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
    inputs.positionDeg = gyro.getYaw(); // degrees
    inputs.pitchDeg = gyro.getPitch();
    inputs.rollDeg = gyro.getRoll();
    inputs.velocityDegPerSec = xyzDps[2]; // degrees per second
  }
}
