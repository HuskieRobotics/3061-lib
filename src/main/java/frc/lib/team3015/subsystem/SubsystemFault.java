package frc.lib.team3015.subsystem;

import edu.wpi.first.wpilibj.Timer;
import java.util.Objects;

public class SubsystemFault {
  public final String description;
  public final double timestamp;
  public final boolean isWarning;

  public SubsystemFault(String description) {
    this(description, false);
  }

  public SubsystemFault(String description, boolean isWarning) {
    this.description = description;
    this.timestamp = Timer.getTimestamp();
    this.isWarning = isWarning;
  }

  @Override
  public boolean equals(Object other) {
    if (this == other) {
      return true;
    }

    if (other == null) {
      return false;
    }

    if (getClass() != other.getClass()) {
      return false;
    }

    SubsystemFault otherSubsystemFault = (SubsystemFault) other;

    return description.equals(otherSubsystemFault.description)
        && isWarning == otherSubsystemFault.isWarning;
  }

  @Override
  public int hashCode() {
    return Objects.hash(description, timestamp, isWarning);
  }
}
