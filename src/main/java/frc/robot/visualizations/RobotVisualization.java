package frc.robot.visualizations;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.leds.LEDs;
import frc.robot.subsystems.elevator.Elevator;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class RobotVisualization {

  // Elevator: White

  private Elevator elevator;

  private LoggedMechanism2d visualization2D;

  private LoggedMechanismRoot2d ledDisplayRoot;
  private LoggedMechanismLigament2d ledDisplayBox;

  private final double kElevatorPosY = 0.51;
  private final double kElevatorStageLength = 0.5;
  private LoggedMechanismRoot2d elevatorRoot;

  private LoggedMechanismLigament2d elevatorStage1;
  private LoggedMechanismLigament2d elevatorStage2;
  private LoggedMechanismLigament2d elevatorStage3;

  public RobotVisualization(Elevator elevator) {
    this.elevator = elevator;
    if (RobotBase.isReal()) return;
    init2dVisualization();
  }

  private void init2dVisualization() {
    visualization2D =
        new LoggedMechanism2d(
            RobotConfig.getInstance().getRobotLengthWithBumpers().in(Meters), 3.0);

    ledDisplayRoot =
        visualization2D.getRoot(
            "ledDisplay",
            RobotConfig.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0,
            2.0); // this goes above the robot
    ledDisplayBox =
        new LoggedMechanismLigament2d("LED State", 0.5, 0, 5.0, new Color8Bit(Color.kBlack));
    ledDisplayRoot.append(ledDisplayBox);

    elevatorRoot =
        visualization2D.getRoot(
            "elevatorRoot",
            RobotConfig.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0,
            kElevatorPosY);
    elevatorStage1 =
        new LoggedMechanismLigament2d("stage1", 0.0, 90.0, 20.0, new Color8Bit(Color.kWhite));
    elevatorStage2 =
        new LoggedMechanismLigament2d("stage2", 0.0, 0.0, 20.0, new Color8Bit(Color.kBlue));
    elevatorStage3 =
        new LoggedMechanismLigament2d("stage3", 0.0, 0.0, 20.0, new Color8Bit(Color.kRed));
    elevatorRoot.append(this.elevatorStage1);
    elevatorStage1.append(this.elevatorStage2);
    elevatorStage2.append(this.elevatorStage3);
  }

  public void update() {
    if (RobotBase.isReal()) return;

    ledDisplayBox.setColor(LEDs.getInstance().getColor(0));

    double elevatorHeightM = this.elevator.getPosition().in(Meters);
    double remainingHeightM = elevatorHeightM;

    // Update elevator stages based on height
    double stage1Length = Math.min(kElevatorStageLength, remainingHeightM);
    remainingHeightM -= stage1Length;
    double stage2Length = Math.min(kElevatorStageLength, remainingHeightM);
    remainingHeightM -= stage2Length;
    double stage3Length = remainingHeightM;

    this.elevatorStage1.setLength(stage1Length);
    this.elevatorStage2.setLength(stage2Length);
    this.elevatorStage3.setLength(stage3Length);

    Logger.recordOutput("Visualization/Mechanisms2D", this.visualization2D);

    // model_0 is elevator bottom cascade
    // model_1 is elevator middle cascade
    // model_2 is elevator top cascade
    double baseHeight = 0.0;
    double middleHeight = Math.max(0.0, elevatorHeightM - 0.631825);
    double topHeight = elevatorHeightM;

    Pose3d elevatorBasePose3d = new Pose3d(0, 0, baseHeight, new Rotation3d(0, 0, 0));
    Pose3d elevatorMiddlePose3d = new Pose3d(0, 0, middleHeight, new Rotation3d(0, 0, 0));
    Pose3d elevatorTopPose3d = new Pose3d(0, 0, topHeight, new Rotation3d(0, 0, 0));

    Logger.recordOutput(
        "Visualization/ComponentsPoseArray",
        new Pose3d[] {elevatorBasePose3d, elevatorMiddlePose3d, elevatorTopPose3d});
  }
}
