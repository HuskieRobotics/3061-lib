// derived from FRC Team 3015's AdvancedSubsystem framework

package first.lib.subsystem.selfcheck;

import first.lib.subsystem.SubsystemFault;
import java.util.List;

public interface SelfChecking {
  List<SubsystemFault> checkForFaults();

  void clearStickyFaults();
}
