// Copyright (c) 2023 FRC 254
// https://github.com/Team254/FRC-2023-Public
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.team254;

import com.ctre.phoenix6.configs.*;
import edu.wpi.first.math.MathUtil;

@java.lang.SuppressWarnings({"java:S106"})
public class TalonFXConfigEquality {

  public static final boolean ENABLE_LOGGING_INEQUALITY = true;
  public static final double TALON_CONFIG_EPSILON = 0.05;

  /**
   * Returns true if the two specified TalonFX configurations are equal; otherwise, false.
   *
   * @param a first TalonFX configuration to compare
   * @param b second TalonFX configuration to compare
   * @return true if the two specified TalonFX configurations are equal; otherwise, false
   */
  public static boolean isEqual(TalonFXConfiguration a, TalonFXConfiguration b) {
    return isEqual(a.Slot0, b.Slot0)
        && isEqual(a.Slot1, b.Slot1)
        && isEqual(a.Slot2, b.Slot2)
        && isEqual(a.MotorOutput, b.MotorOutput)
        && isEqual(a.CurrentLimits, b.CurrentLimits)
        && isEqual(a.Voltage, b.Voltage)
        && isEqual(a.TorqueCurrent, b.TorqueCurrent)
        && isEqual(a.Feedback, b.Feedback)
        && isEqual(a.OpenLoopRamps, b.OpenLoopRamps)
        && isEqual(a.ClosedLoopRamps, b.ClosedLoopRamps)
        && isEqual(a.HardwareLimitSwitch, b.HardwareLimitSwitch)
        && isEqual(a.Audio, b.Audio)
        && isEqual(a.SoftwareLimitSwitch, b.SoftwareLimitSwitch)
        && isEqual(a.MotionMagic, b.MotionMagic);
  }

  /**
   * Returns true if the two specified slot 0 configurations are equal; otherwise, false.
   *
   * @param a first slot 0 configuration to compare
   * @param b second slot 0 configuration to compare
   * @return true if the two specified slot 0 configurations are equal; otherwise, false
   */
  public static boolean isEqual(Slot0Configs a, Slot0Configs b) {
    boolean val =
        MathUtil.isNear(a.kP, b.kP, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(a.kI, b.kI, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(a.kD, b.kD, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(a.kV, b.kV, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(a.kS, b.kS, TALON_CONFIG_EPSILON);
    if (ENABLE_LOGGING_INEQUALITY && !val) {
      System.out.println("Slot0Configs not equal");
    }
    return val;
  }

  /**
   * Returns true if the two specified slot 1 configurations are equal; otherwise, false.
   *
   * @param a first slot 1 configuration to compare
   * @param b second slot 1 configuration to compare
   * @return true if the two specified slot 1 configurations are equal; otherwise, false
   */
  public static boolean isEqual(Slot1Configs a, Slot1Configs b) {
    boolean val =
        MathUtil.isNear(a.kP, b.kP, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(a.kI, b.kI, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(a.kD, b.kD, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(a.kV, b.kV, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(a.kS, b.kS, TALON_CONFIG_EPSILON);
    if (ENABLE_LOGGING_INEQUALITY && !val) {
      System.out.println("Slot1Configs not equal");
    }
    return val;
  }

  /**
   * Returns true if the two specified slot 2 configurations are equal; otherwise, false.
   *
   * @param a first slot 2 configuration to compare
   * @param b second slot 2 configuration to compare
   * @return true if the two specified slot 2 configurations are equal; otherwise, false
   */
  public static boolean isEqual(Slot2Configs a, Slot2Configs b) {
    boolean val =
        MathUtil.isNear(a.kP, b.kP, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(a.kI, b.kI, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(a.kD, b.kD, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(a.kV, b.kV, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(a.kS, b.kS, TALON_CONFIG_EPSILON);
    if (ENABLE_LOGGING_INEQUALITY && !val) {
      System.out.println("Slot2Configs not equal");
    }
    return val;
  }

  /**
   * Returns true if the two specified motor output configurations are equal; otherwise, false.
   *
   * @param a first motor output configuration to compare
   * @param b second motor output configuration to compare
   * @return true if the two specified motor output configurations are equal; otherwise, false
   */
  public static boolean isEqual(MotorOutputConfigs a, MotorOutputConfigs b) {
    boolean val =
        a.Inverted.value == b.Inverted.value
            && a.NeutralMode.value == b.NeutralMode.value
            && MathUtil.isNear(
                a.DutyCycleNeutralDeadband, b.DutyCycleNeutralDeadband, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(a.PeakForwardDutyCycle, b.PeakForwardDutyCycle, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(
                a.PeakReverseDutyCycle, b.PeakReverseDutyCycle, TALON_CONFIG_EPSILON);
    if (ENABLE_LOGGING_INEQUALITY && !val) {
      System.out.println("MotorOutputConfigs not equal");
    }
    return val;
  }

  /**
   * Returns true if the two specified current limits configurations are equal; otherwise, false.
   *
   * @param a first current limits configuration to compare
   * @param b second current limits configuration to compare
   * @return true if the two specified current limits configurations are equal; otherwise, false
   */
  public static boolean isEqual(CurrentLimitsConfigs a, CurrentLimitsConfigs b) {
    boolean val =
        MathUtil.isNear(a.StatorCurrentLimit, b.StatorCurrentLimit, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(a.SupplyCurrentLimit, b.SupplyCurrentLimit, TALON_CONFIG_EPSILON)
            && a.StatorCurrentLimitEnable == b.StatorCurrentLimitEnable
            && a.SupplyCurrentLimitEnable == b.SupplyCurrentLimitEnable;
    if (ENABLE_LOGGING_INEQUALITY && !val) {
      System.out.println("CurrentLimitsConfigs not equal");
    }
    return val;
  }

  /**
   * Returns true if the two specified voltage configurations are equal; otherwise, false.
   *
   * @param a first voltage configuration to compare
   * @param b second voltage configuration to compare
   * @return true if the two specified voltage configurations are equal; otherwise, false
   */
  public static boolean isEqual(VoltageConfigs a, VoltageConfigs b) {
    boolean val =
        MathUtil.isNear(
                a.SupplyVoltageTimeConstant, b.SupplyVoltageTimeConstant, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(a.PeakForwardVoltage, b.PeakForwardVoltage, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(a.PeakReverseVoltage, b.PeakReverseVoltage, TALON_CONFIG_EPSILON);
    if (ENABLE_LOGGING_INEQUALITY && !val) {
      System.out.println("VoltageConfigs not equal");
    }
    return val;
  }

  /**
   * Returns true if the two specified torque current configurations are equal; otherwise, false.
   *
   * @param a first torque current configuration to compare
   * @param b second torque current configuration to compare
   * @return true if the two specified torque current configurations are equal; otherwise, false
   */
  public static boolean isEqual(TorqueCurrentConfigs a, TorqueCurrentConfigs b) {
    boolean val =
        MathUtil.isNear(
                a.PeakForwardTorqueCurrent, b.PeakForwardTorqueCurrent, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(
                a.PeakReverseTorqueCurrent, b.PeakReverseTorqueCurrent, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(
                a.TorqueNeutralDeadband, b.TorqueNeutralDeadband, TALON_CONFIG_EPSILON);
    if (ENABLE_LOGGING_INEQUALITY && !val) {
      System.out.println("TorqueCurrentConfigs not equal");
    }
    return val;
  }

  /**
   * Returns true if the two specified feedback configurations are equal; otherwise, false.
   *
   * @param a first feedback configuration to compare
   * @param b second feedback configuration to compare
   * @return true if the two specified feedback configurations are equal; otherwise, false
   */
  public static boolean isEqual(FeedbackConfigs a, FeedbackConfigs b) {
    boolean val =
        MathUtil.isNear(a.FeedbackRotorOffset, b.FeedbackRotorOffset, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(
                a.SensorToMechanismRatio, b.SensorToMechanismRatio, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(a.RotorToSensorRatio, b.RotorToSensorRatio, TALON_CONFIG_EPSILON)
            && a.FeedbackSensorSource.value == b.FeedbackSensorSource.value
            && a.FeedbackRemoteSensorID == b.FeedbackRemoteSensorID;
    if (ENABLE_LOGGING_INEQUALITY && !val) {
      System.out.println("FeedbackConfigs not equal");
      System.out.printf(
          "A-FeedbackRotorOffset: %f, B-FeedbackRotorOffset: %f%n",
          a.FeedbackRotorOffset, b.FeedbackRotorOffset);
      System.out.printf(
          "A-SensorToMechanismRatio: %f, B-SensorToMechanismRatio: %f%n",
          a.SensorToMechanismRatio, b.SensorToMechanismRatio);
      System.out.printf(
          "A-RotorToSensorRatio: %f, B-RotorToSensorRatio: %f%n",
          a.RotorToSensorRatio, b.RotorToSensorRatio);
      System.out.printf(
          "A-FeedbackSensorSource: %d, B-FeedbackSensorSource: %d%n",
          a.FeedbackSensorSource.value, b.FeedbackSensorSource.value);
      System.out.printf(
          "A-FeedbackRemoteSensorID: %d, B-FeedbackRemoteSensorID: %d%n",
          a.FeedbackRemoteSensorID, b.FeedbackRemoteSensorID);
    }
    return val;
  }

  /**
   * Returns true if the two specified open loop ramps configurations are equal; otherwise, false.
   *
   * @param a first open loop ramps configuration to compare
   * @param b second open loop ramps configuration to compare
   * @return true if the two specified open loop ramps configurations are equal; otherwise, false
   */
  public static boolean isEqual(OpenLoopRampsConfigs a, OpenLoopRampsConfigs b) {
    boolean val =
        MathUtil.isNear(
                a.DutyCycleOpenLoopRampPeriod, b.DutyCycleOpenLoopRampPeriod, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(
                a.VoltageOpenLoopRampPeriod, b.VoltageOpenLoopRampPeriod, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(
                a.TorqueOpenLoopRampPeriod, b.TorqueOpenLoopRampPeriod, TALON_CONFIG_EPSILON);
    if (ENABLE_LOGGING_INEQUALITY && !val) {
      System.out.println("OpenLoopRampsConfigs not equal");
    }
    return val;
  }

  /**
   * Returns true if the two specified closed loop ramps configurations are equal; otherwise, false.
   *
   * @param a first closed loop ramps configuration to compare
   * @param b second closed loop ramps configuration to compare
   * @return true if the two specified closed loop ramps configurations are equal; otherwise, false
   */
  public static boolean isEqual(ClosedLoopRampsConfigs a, ClosedLoopRampsConfigs b) {
    boolean val =
        MathUtil.isNear(
                a.DutyCycleClosedLoopRampPeriod,
                b.DutyCycleClosedLoopRampPeriod,
                TALON_CONFIG_EPSILON)
            && MathUtil.isNear(
                a.VoltageClosedLoopRampPeriod, b.VoltageClosedLoopRampPeriod, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(
                a.TorqueClosedLoopRampPeriod, b.TorqueClosedLoopRampPeriod, TALON_CONFIG_EPSILON);
    if (ENABLE_LOGGING_INEQUALITY && !val) {
      System.out.println("ClosedLoopRampsConfigs not equal");
    }
    return val;
  }

  /**
   * Returns true if the two specified hardware limit switch configurations are equal; otherwise,
   * false.
   *
   * @param a first hardware limit switch configuration to compare
   * @param b second hardware limit switch configuration to compare
   * @return true if the two specified hardware limit switch configurations are equal; otherwise,
   *     false
   */
  public static boolean isEqual(HardwareLimitSwitchConfigs a, HardwareLimitSwitchConfigs b) {
    boolean val =
        a.ForwardLimitAutosetPositionEnable == b.ForwardLimitAutosetPositionEnable
            && a.ForwardLimitEnable == b.ForwardLimitEnable
            && a.ReverseLimitAutosetPositionEnable == b.ReverseLimitAutosetPositionEnable
            && a.ReverseLimitEnable == b.ReverseLimitEnable
            && MathUtil.isNear(
                a.ForwardLimitAutosetPositionValue,
                b.ForwardLimitAutosetPositionValue,
                TALON_CONFIG_EPSILON)
            && MathUtil.isNear(
                a.ReverseLimitAutosetPositionValue,
                b.ReverseLimitAutosetPositionValue,
                TALON_CONFIG_EPSILON)
            && a.ForwardLimitRemoteSensorID == b.ForwardLimitRemoteSensorID
            && a.ReverseLimitRemoteSensorID == b.ReverseLimitRemoteSensorID
            && a.ForwardLimitSource.value == b.ForwardLimitSource.value
            && a.ReverseLimitSource.value == b.ReverseLimitSource.value
            && a.ForwardLimitType.value == b.ForwardLimitType.value
            && a.ReverseLimitType.value == b.ReverseLimitType.value;
    if (ENABLE_LOGGING_INEQUALITY && !val) {
      System.out.println("HardwareLimitSwitchConfigs not equal");
    }
    return val;
  }

  /**
   * Returns true if the two specified audio configurations are equal; otherwise, false.
   *
   * @param a first audio configuration to compare
   * @param b second audio configuration to compare
   * @return true if the two specified audio configurations are equal; otherwise, false
   */
  public static boolean isEqual(AudioConfigs a, AudioConfigs b) {
    boolean val = a.BeepOnBoot == b.BeepOnBoot;
    if (ENABLE_LOGGING_INEQUALITY && !val) {
      System.out.println("AudioConfigs not equal");
    }
    return val;
  }

  /**
   * Returns true if the two specified software limit switch configurations are equal; otherwise,
   *
   * @param a first software limit switch configuration to compare
   * @param b second software limit switch configuration to compare
   * @return true if the two specified software limit switch configurations are equal; otherwise,
   */
  public static boolean isEqual(SoftwareLimitSwitchConfigs a, SoftwareLimitSwitchConfigs b) {
    boolean val =
        MathUtil.isNear(
                a.ForwardSoftLimitThreshold, b.ForwardSoftLimitThreshold, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(
                a.ReverseSoftLimitThreshold, b.ReverseSoftLimitThreshold, TALON_CONFIG_EPSILON)
            && a.ReverseSoftLimitEnable == b.ReverseSoftLimitEnable
            && a.ForwardSoftLimitEnable == b.ForwardSoftLimitEnable;
    if (ENABLE_LOGGING_INEQUALITY && !val) {
      System.out.println("SoftwareLimitSwitchConfigs not equal");
    }
    return val;
  }

  /**
   * Returns true if the two specified motion magic configurations are equal; otherwise, false.
   *
   * @param a first motion magic configuration to compare
   * @param b second motion magic configuration to compare
   * @return true if the two specified motion magic configurations are equal; otherwise, false
   */
  public static boolean isEqual(MotionMagicConfigs a, MotionMagicConfigs b) {
    boolean val =
        MathUtil.isNear(a.MotionMagicAcceleration, b.MotionMagicAcceleration, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(
                a.MotionMagicCruiseVelocity, b.MotionMagicCruiseVelocity, TALON_CONFIG_EPSILON)
            && MathUtil.isNear(a.MotionMagicJerk, b.MotionMagicJerk, TALON_CONFIG_EPSILON);
    if (ENABLE_LOGGING_INEQUALITY && !val) {
      System.out.println("MotionMagicConfigs not equal");
    }
    return val;
  }

  private TalonFXConfigEquality() {}
}
