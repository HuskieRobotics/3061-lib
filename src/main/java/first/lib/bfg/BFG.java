package first.lib.bfg;

import first.lib.RobotConfig;
import first.lib.energy.EnergyMonitor;
import first.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.RobotBase;
import org.wpilib.util.Alert;

/**
 * Logs data from the Playing With Fusion Battery Fuel Gauge (BFG).
 *
 * <p>The BFG logs:
 *
 * <ul>
 *   <li>voltage, current, depth of discharge, and remaining charge every robot loop
 *   <li>battery and charge-cycle data once, after the sensor has been connected long enough for all
 *       of the once-per-second frames that contain this data to have been received
 *   <li>match data once, when the robot is disabled after teleop while connected to the FMS
 * </ul>
 *
 * <p>Like {@link first.lib.leds.LEDs}, the BFG is a singleton subsystem so that its data is
 * available throughout the robot code. No commands require it; it is a subsystem so that the
 * command scheduler invokes its periodic method every robot loop. The scheduler invokes subsystems'
 * periodic methods in the order that the subsystems were created. So, the BFG should be created (by
 * invoking {@link #getInstance()}) immediately after the robot's config and before any other
 * subsystems, so that other subsystems have the current loop's battery data.
 */
public class BFG extends SubsystemBase {

  private static final String SUBSYSTEM_NAME = "BFG";

  // the BFG sends its battery, charge-cycle, and match data once per second; so, wait longer than
  // that after the BFG connects before reading the battery and charge-cycle data
  private static final double BATTERY_INPUTS_DELAY_SECS = 2.0;

  private static final Alert invalidCANIDAlert =
      new Alert(
          SUBSYSTEM_NAME,
          "BFG CAN ID must be between 0 and 63; the BFG will not be read",
          Alert.Level.HIGH);

  private static BFG instance;

  /**
   * Returns the singleton instance of this class, creating it on the first call. When running on a
   * real robot whose config specifies a BFG CAN ID ({@link RobotConfig#getBFGCANID()}), the BFG is
   * read from the CAN port specified by {@link RobotConfig#getBFGCANPort()}. When simulating such a
   * robot, the BFG and battery are simulated by {@link BFGIOSim}.
   *
   * @return the singleton instance of this class
   */
  public static BFG getInstance() {
    if (instance == null) {
      BFGIO io = new BFGIO() {};
      int canID = RobotConfig.getInstance().getBFGCANID();
      if (Constants.getMode() == Constants.Mode.SIM && canID != -1) {
        // the simulated BFG draws the simulated motors' current, as measured by the energy monitor
        io = new BFGIOSim(() -> EnergyMonitor.getInstance().getMotorAmps());
      } else if (Constants.getMode() == Constants.Mode.REAL && canID != -1) {
        if (canID >= 0 && canID <= 63) {
          // TODO: replace BFGIOCAN with an IO class that uses Playing With Fusion's vendordep once
          // a 2027 version that supports SystemCore is released (issue #187)
          io = new BFGIOCAN(RobotConfig.getInstance().getBFGCANPort(), canID);
        } else {
          invalidCANIDAlert.set(true);
        }
      }
      instance = new BFG(io);
    }
    return instance;
  }

  private final BFGIO io;
  private final BFGIOInputsAutoLogged inputs = new BFGIOInputsAutoLogged();
  private final BFGBatteryIOInputsAutoLogged batteryInputs = new BFGBatteryIOInputsAutoLogged();
  private final BFGMatchIOInputsAutoLogged matchInputs = new BFGMatchIOInputsAutoLogged();

  private boolean hasReadBatteryInputs = false;
  private long connectedSinceNanos = -1;
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
   * are read at the same point in the log at which they were recorded, since the decision to read
   * them depends only on logged inputs, the logged timestamp, and the logged robot state.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(SUBSYSTEM_NAME, inputs);

    // read the battery and charge-cycle data once the BFG has been connected long enough for all of
    // the once-per-second frames that contain it to have been received
    long timestampNanos = Logger.getTimestamp();
    if (!inputs.connected) {
      connectedSinceNanos = -1;
    } else if (connectedSinceNanos < 0) {
      connectedSinceNanos = timestampNanos;
    }
    if (!hasReadBatteryInputs
        && connectedSinceNanos >= 0
        && (timestampNanos - connectedSinceNanos) / 1.0e9 >= BATTERY_INPUTS_DELAY_SECS) {
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
