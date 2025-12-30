package frc.lib.team254;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class ChezyRepeatCommand extends Command {
  private final Command m_command;
  private boolean m_ended;
  private final int kMaxLoops = 3;

  /**
   * Creates a new RepeatCommand. Will run another command repeatedly, restarting it whenever it
   * ends, until this command is interrupted.
   *
   * @param command the command to run repeatedly
   */
  @SuppressWarnings("this-escape")
  public ChezyRepeatCommand(Command command) {
    m_command = requireNonNullParam(command, "command", "RepeatCommand");
    CommandScheduler.getInstance().registerComposedCommands(command);
    addRequirements(command.getRequirements());
    setName("Repeat(" + command.getName() + ")");
  }

  @Override
  public void initialize() {
    m_ended = false;
    m_command.initialize();
  }

  @Override
  public void execute() {
    int loops = 0;
    while (loops < kMaxLoops) {
      if (m_ended) {
        m_ended = false;
        m_command.initialize();
      }
      m_command.execute();
      if (m_command.isFinished()) {
        // restart command
        m_command.end(false);
        m_ended = true;
      } else {
        return;
      }
      loops++;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // Make sure we didn't already call end() (which would happen if the command finished in the
    // last call to our execute())
    if (!m_ended) {
      m_command.end(interrupted);
      m_ended = true;
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return m_command.runsWhenDisabled();
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return m_command.getInterruptionBehavior();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty("command", m_command::getName, null);
  }
}
