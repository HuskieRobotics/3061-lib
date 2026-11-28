// derived from FRC Team 3015's AdvancedSubsystem framework

package first.lib.subsystem.selfcheck;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import first.lib.subsystem.SubsystemFault;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingSparkMax implements SelfChecking {
  private final String label;
  private SparkMax spark;

  public SelfCheckingSparkMax(String label, SparkMax spark) {
    this.label = label;
    this.spark = spark;
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    ArrayList<SubsystemFault> faults = new ArrayList<>();

    REVLibError err = spark.getLastError();
    if (err != REVLibError.kOk) {
      faults.add(new SubsystemFault(String.format("[%s]: Error: %s", label, err.name())));
    }

    return faults;
  }

  @Override
  public void clearStickyFaults() {
    spark.clearFaults();
  }
}
