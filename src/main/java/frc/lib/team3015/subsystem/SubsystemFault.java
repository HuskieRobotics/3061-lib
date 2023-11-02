package frc.lib.team3015.subsystem;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.team6328.util.Alert;
import java.util.Objects;

public class SubsystemFault {
  public final String description;
  public final double timestamp;
  public final boolean isWarning;
  public final boolean sticky;
  public final Alert alert;

  public SubsystemFault(String description, boolean isWarning) {
    // default sticky to true
    this(description, isWarning, true);
  }

  public SubsystemFault(String description) {
    this(description, false);
  }

  // what does sticky mean specifically?
  public SubsystemFault(String description, boolean isWarning, boolean sticky) {
    this.description = description;
    this.timestamp = Timer.getFPGATimestamp();
    this.isWarning = isWarning;
    this.sticky = sticky;
    this.alert =
        new Alert(description, isWarning ? Alert.AlertType.WARNING : Alert.AlertType.ERROR);
    this.alert.set(true);
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
    return Objects.hash(description, timestamp, isWarning, sticky);
  }
}
