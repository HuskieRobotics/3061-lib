package frc.robot.visualizations;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.team3061.RobotConfig;
import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.climber.Climber;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class RobotVisualization {

  private Intake intake;
  // private Climber climber;

  private LoggedMechanism2d intakeVisualization2D;

  private LoggedMechanismLigament2d ledDisplayBox;
  private final double kLEDHeight = Units.inchesToMeters(12.75);
  private final double kLEDLength = Units.inchesToMeters(6.0);

  private LoggedMechanismLigament2d intakeBox;
  private final double kIntakeRootPosX = Units.inchesToMeters(17.25);
  private final double kIntakeRootPosY = Units.inchesToMeters(7.00);
  private final double kIntakeDepth = Units.inchesToMeters(11.0);

  private LoggedMechanism2d climberVisualization2D;

  private LoggedMechanismLigament2d climberBox;
  private final double kClimberRootPosY = Units.inchesToMeters(4.875);
  private final double kClimberRootPosZ = Units.inchesToMeters(20.25);
  private final double kClimberLength = Units.inchesToMeters(10.0);

  public RobotVisualization(Intake intake) {
    if (RobotBase.isReal()) return;

    this.intake = intake;
    init2dVisualization();
  }

  private void init2dVisualization() {
    intakeVisualization2D =
        new LoggedMechanism2d(RobotConfig.getInstance().getRobotLengthWithBumpersMeters(), 3.0);

    // LEDs
    LoggedMechanismRoot2d ledDisplayRoot =
        intakeVisualization2D.getRoot("ledDisplay", 0.0, kLEDHeight); // this goes above the robot
    ledDisplayBox =
        new LoggedMechanismLigament2d(
            "LED State", kLEDLength, 0.0, 1.0, new Color8Bit(Color.kBlack));
    ledDisplayRoot.append(ledDisplayBox);

    // Intake
    LoggedMechanismRoot2d intakeRoot =
        intakeVisualization2D.getRoot("intakeRoot", kIntakeRootPosX, kIntakeRootPosY);
    intakeBox =
        new LoggedMechanismLigament2d(
            "intake", kIntakeDepth, 0.0, 80.0, new Color8Bit(Color.kWhite));
    intakeRoot.append(this.intakeBox);
    climberVisualization2D =
        new LoggedMechanism2d(RobotConfig.getInstance().getRobotLengthWithBumpersMeters(), 3.0);

    // Climber
    LoggedMechanismRoot2d climberRoot =
        climberVisualization2D.getRoot("climberRoot", kClimberRootPosY, kClimberRootPosZ);
    climberBox =
        new LoggedMechanismLigament2d(
            "climber", kClimberLength, 0.0, 5.0, new Color8Bit(Color.kOrange));
    climberRoot.append(this.climberBox);
  }

  public void update() {
    if (RobotBase.isReal()) return;

    // climberBox.setAngle(climber.getAngle().in(Degrees));

    if (intake.isDeployed()) {
      intakeBox.setLength(kIntakeDepth * 2.0);
    } else {
      intakeBox.setLength(kIntakeDepth);
    }

    Logger.recordOutput("Visualization/Intake", this.intakeVisualization2D);
    Logger.recordOutput("Visualization/Climber", this.climberVisualization2D);
  }
}
