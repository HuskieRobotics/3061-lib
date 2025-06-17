/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.lib.team3061.gyro;

import static edu.wpi.first.units.Units.*;
import static frc.lib.team3061.drivetrain.DrivetrainConstants.SUBSYSTEM_NAME;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3061.RobotConfig;
import frc.robot.Constants;

public class GyroIOPigeon2Phoenix6 implements GyroIO {

  private static final double SIGNAL_UPDATE_FREQUENCY_HZ = 100.0;

  private final Pigeon2 gyro;
  private final StatusSignal<Angle> yawStatusSignal;
  private final StatusSignal<Angle> pitchStatusSignal;
  private final StatusSignal<Angle> rollStatusSignal;
  private final StatusSignal<AngularVelocity> angularVelocityXStatusSignal;
  private final StatusSignal<AngularVelocity> angularVelocityYStatusSignal;
  private final StatusSignal<AngularVelocity> angularVelocityZStatusSignal;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  private final Pigeon2SimState gyroSim;

  private final Alert refreshAlert =
      new Alert("Failed to refresh signals in " + SUBSYSTEM_NAME, AlertType.kError);

  public GyroIOPigeon2Phoenix6(int id) {
    gyro = new Pigeon2(id, RobotConfig.getInstance().getCANBusName());
    this.yawStatusSignal = this.gyro.getYaw();
    this.yawStatusSignal.setUpdateFrequency(SIGNAL_UPDATE_FREQUENCY_HZ);
    this.pitchStatusSignal = this.gyro.getPitch();
    this.pitchStatusSignal.setUpdateFrequency(SIGNAL_UPDATE_FREQUENCY_HZ);
    this.rollStatusSignal = this.gyro.getRoll();
    this.rollStatusSignal.setUpdateFrequency(SIGNAL_UPDATE_FREQUENCY_HZ);
    this.angularVelocityXStatusSignal = this.gyro.getAngularVelocityXWorld();
    this.angularVelocityXStatusSignal.setUpdateFrequency(SIGNAL_UPDATE_FREQUENCY_HZ);
    this.angularVelocityYStatusSignal = this.gyro.getAngularVelocityYWorld();
    this.angularVelocityYStatusSignal.setUpdateFrequency(SIGNAL_UPDATE_FREQUENCY_HZ);
    this.angularVelocityZStatusSignal = this.gyro.getAngularVelocityZWorld();
    this.angularVelocityZStatusSignal.setUpdateFrequency(SIGNAL_UPDATE_FREQUENCY_HZ);

    // To improve performance, register all signals with Phoenix6Util. All signals on the entire CAN
    // bus will be refreshed at the same time by Phoenix6Util; so, there is no need to refresh any
    // StatusSignals in this class.
    Phoenix6Util.registerSignals(
        true,
        this.yawStatusSignal,
        this.angularVelocityZStatusSignal,
        this.pitchStatusSignal,
        this.rollStatusSignal,
        this.angularVelocityXStatusSignal,
        this.angularVelocityYStatusSignal);

    if (Constants.getMode() == Constants.Mode.SIM) {
      this.gyroSim = this.gyro.getSimState();
    } else {
      this.gyroSim = null;
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // Determine if the Pigeon is still connected (i.e., reachable on the CAN bus). We do this by
    // verifying that none of the status signals for the device report an error.
    inputs.connected =
        connectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                this.yawStatusSignal,
                this.angularVelocityZStatusSignal,
                this.pitchStatusSignal,
                this.rollStatusSignal,
                this.angularVelocityXStatusSignal,
                this.angularVelocityYStatusSignal));
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

    // The last step in the updateInputs method is to update the simulation.
    if (Constants.getMode() == Constants.Mode.SIM) {
      this.gyroSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    }
  }
}
