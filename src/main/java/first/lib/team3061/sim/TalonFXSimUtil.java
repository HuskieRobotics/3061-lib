package first.lib.team3061.sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

/** Helper methods shared by the system sim classes that simulate TalonFX motors. */
final class TalonFXSimUtil {

  private TalonFXSimUtil() {}

  /**
   * Gets the sim state of each motor and sets its orientation such that a positive rotor position
   * and velocity in the sim corresponds to the positive direction of the mechanism.
   *
   * <p>A motor that is following another motor ignores its own Inverted config. Its direction is
   * determined by the leader's Inverted config and the Follower request's MotorAlignment.
   * Therefore, the orientation of a follower is derived from its leader, which must also be one of
   * the specified motors. The Follower control request must be applied to each follower before
   * invoking this method.
   *
   * @param motors the motors that drive the mechanism; the first motor must be the leader
   * @return the sim states of the specified motors, in the same order
   */
  static TalonFXSimState[] createSimStates(TalonFX... motors) {
    if (motors.length == 0) {
      throw new IllegalArgumentException("at least one motor must be specified");
    }

    TalonFXSimState[] simStates = new TalonFXSimState[motors.length];
    for (int i = 0; i < motors.length; i++) {
      simStates[i] = motors[i].getSimState();
      simStates[i].Orientation =
          isClockwisePositive(motors[i], motors)
              ? ChassisReference.Clockwise_Positive
              : ChassisReference.CounterClockwise_Positive;
    }
    return simStates;
  }

  private static boolean isClockwisePositive(TalonFX motor, TalonFX[] motors) {
    if (motor.getAppliedControl() instanceof Follower follower) {
      for (TalonFX leader : motors) {
        if (leader.getDeviceID() == follower.LeaderID) {
          boolean opposed = follower.MotorAlignment == MotorAlignmentValue.Opposed;
          return isConfiguredClockwisePositive(leader) != opposed;
        }
      }
      throw new IllegalArgumentException(
          "the leader (ID "
              + follower.LeaderID
              + ") of the follower (ID "
              + motor.getDeviceID()
              + ") must also be specified");
    }
    return isConfiguredClockwisePositive(motor);
  }

  private static boolean isConfiguredClockwisePositive(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    motor.getConfigurator().refresh(config);
    return config.MotorOutput.Inverted == InvertedValue.Clockwise_Positive;
  }
}
