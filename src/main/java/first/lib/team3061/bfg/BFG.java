package first.lib.team3061.bfg;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.RobotBase;

/**
 * Logs data from the Playing With Fusion Battery Fuel Gauge (BFG).
 *
 * <p>The BFG logs:
 *
 * <ul>
 *   <li>voltage, current, depth of discharge, and remaining charge every robot loop
 *   <li>battery and charge-cycle data once, when the sensor first connects
 *   <li>match data once, when the robot is disabled after teleop while connected to the FMS
 * </ul>
 *
 * <p>Like {@link first.lib.team3061.leds.LEDs}, the BFG is a singleton subsystem so that its data
 * is available throughout the robot code. No commands require it; it is a subsystem so that the
 * command scheduler invokes its periodic method every robot loop. The scheduler invokes subsystems'
 * periodic methods in the order that the subsystems were created. So, the BFG should be created (by
 * invoking {@link #getInstance()}) immediately after the robot's config and before any other
 * subsystems, so that other subsystems have the current loop's battery data.
 */
public class BFG extends SubsystemBase {

  private static final String SUBSYSTEM_NAME = "BFG";

  private static BFG instance;

  /**
   * Returns the singleton instance of this class, creating it on the first call with the CAN ID
   * specified by the robot's config.
   *
   * @return the singleton instance of this class
   */
  public static BFG getInstance() {
    if (instance == null) {
      // TODO: when running on a real robot whose config specifies a BFG CAN ID
      // (RobotConfig.getBFGCANID), create a BFGIOPlayingWithFusion with that CAN ID. This requires
      // a 2027 Playing With Fusion vendordep that supports SystemCore, which hasn't been published.
      // Until then, the BFG logs only its disconnected default values.
      instance = new BFG(new BFGIO() {});
    }
    return instance;
  }

  private final BFGIO io;
  private final BFGIOInputsAutoLogged inputs = new BFGIOInputsAutoLogged();
  private final BFGBatteryIOInputsAutoLogged batteryInputs = new BFGBatteryIOInputsAutoLogged();
  private final BFGMatchIOInputsAutoLogged matchInputs = new BFGMatchIOInputsAutoLogged();

  private boolean hasReadBatteryInputs = false;
  private boolean hasReadMatchInputs = false;
  private boolean hasTeleopStarted = false;

  /**
   * Creates a new BFG that reads data via the specified IO.
   *
   * @param io the IO for the BFG
   */
  private BFG(BFGIO io) {
    super(SUBSYSTEM_NAME);
    this.io = io;
  }

  /**
   * Updates and logs the BFG's inputs. The command scheduler invokes this method every robot loop.
   *
   * <p>The battery and match inputs are each updated and logged once. When replaying a log, they
   * are read at the same point in the log at which they were recorded.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(SUBSYSTEM_NAME, inputs);

    if (!hasReadBatteryInputs && inputs.connected) {
      hasReadBatteryInputs = true;
      io.updateBatteryInputs(batteryInputs);
      Logger.processInputs(SUBSYSTEM_NAME + "/Battery", batteryInputs);
    }

    if (RobotState.isFMSAttached() && RobotBase.isTeleopEnabled()) {
      hasTeleopStarted = true;
    }

    if (!hasReadMatchInputs
        && hasTeleopStarted
        && RobotBase.isDisabled()
        && RobotState.isFMSAttached()
        && inputs.connected) {
      hasReadMatchInputs = true;
      io.updateMatchInputs(matchInputs);
      Logger.processInputs(SUBSYSTEM_NAME + "/Match", matchInputs);
    }
  }

  /**
   * Returns whether the BFG is connected.
   *
   * @return true if the BFG is connected; false otherwise
   */
  public boolean isConnected() {
    return inputs.connected;
  }

  /**
   * Returns the current measured at the battery's terminals in this robot loop. This is only valid
   * if the BFG is connected.
   *
   * @return the current in amps
   */
  public double getCurrentAmps() {
    return inputs.currentAmps;
  }

  /**
   * Returns the voltage measured at the battery's terminals in this robot loop. This is only valid
   * if the BFG is connected.
   *
   * @return the voltage in volts
   */
  public double getVoltageVolts() {
    return inputs.voltageVolts;
  }
}
