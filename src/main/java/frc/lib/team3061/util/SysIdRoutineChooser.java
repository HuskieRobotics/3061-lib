package frc.lib.team3061.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class SysIdRoutineChooser {

  private static final SysIdRoutineChooser instance = new SysIdRoutineChooser();

  private Map<Integer, Command> dynamicForwardRoutines = new HashMap<>();
  private Map<Integer, Command> dynamicReverseRoutines = new HashMap<>();
  private Map<Integer, Command> quasistaticForwardRoutines = new HashMap<>();
  private Map<Integer, Command> quasistaticReverseRoutines = new HashMap<>();

  private final LoggedDashboardChooser<Integer> sysIdChooser =
      new LoggedDashboardChooser<>("SysId Chooser");

  private SysIdRoutineChooser() {}

  public static SysIdRoutineChooser getInstance() {
    return instance;
  }

  public void addOption(String name, SysIdRoutine sysIdRoutine) {
    sysIdChooser.addOption(name, name.hashCode());
    dynamicForwardRoutines.put(name.hashCode(), sysIdRoutine.dynamic(Direction.kForward));
    dynamicReverseRoutines.put(name.hashCode(), sysIdRoutine.dynamic(Direction.kReverse));
    quasistaticForwardRoutines.put(name.hashCode(), sysIdRoutine.quasistatic(Direction.kForward));
    quasistaticReverseRoutines.put(name.hashCode(), sysIdRoutine.quasistatic(Direction.kReverse));
  }

  public Command getDynamicForward() {
    return new SelectCommand<>(this.dynamicForwardRoutines, sysIdChooser::get);
  }

  public Command getDynamicReverse() {
    return new SelectCommand<>(this.dynamicReverseRoutines, sysIdChooser::get);
  }

  public Command getQuasistaticForward() {
    return new SelectCommand<>(this.quasistaticForwardRoutines, sysIdChooser::get);
  }

  public Command getQuasistaticReverse() {
    return new SelectCommand<>(this.quasistaticReverseRoutines, sysIdChooser::get);
  }
}
