package frc.lib.team3015.subsystem;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.team6328.util.Alert;

import java.util.Objects;

public class SubsystemFault{
  public final String description;
  public final double timestamp;
  public final boolean isWarning;
  public final boolean sticky;
  public SubsystemFault(String description, boolean isWarning) {
    this(description, isWarning, false);
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

    if(isWarning){
      Alert alert = new Alert(description, Alert.AlertType.WARNING);

      // is there a specific condition that we should look at for when to make this alert NOT show up?
      alert.set(true);
    }else{
      // if not a warning, is it more serious or does the other one consider a warning to be the most serious?
      
      Alert alert = new Alert(description, Alert.AlertType.ERROR);

      // same here as above
      alert.set(true);
    }
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

