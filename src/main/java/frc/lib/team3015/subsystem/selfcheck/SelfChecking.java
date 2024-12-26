package frc.lib.team3015.subsystem.selfcheck;

import frc.lib.team3015.subsystem.SubsystemFault;
import java.util.List;

public interface SelfChecking {
  List<SubsystemFault> checkForFaults();

  void clearStickyFaults();
}
