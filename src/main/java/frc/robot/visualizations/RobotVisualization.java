package frc.robot.visualizations;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.team3061.leds.LEDs;
import frc.robot.subsystems.elevator.Elevator;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class RobotVisualization {

  // Elevator: White

  private Elevator elevator;

  private LoggedMechanism2d viz2d = new LoggedMechanism2d(1.27, 2.032);

  private LoggedMechanismRoot2d ledDisplayRoot =
      viz2d.getRoot("ledDisplay", 0.58, 1.5); // this goes above the robot
  private LoggedMechanismLigament2d ledDisplayBox =
      new LoggedMechanismLigament2d("LED State", 0.5, 0, 5.0, new Color8Bit(Color.kBlack));

  private Color8Bit blue = new Color8Bit(Color.kBlue);
  private Color8Bit white = new Color8Bit(Color.kWhite);

  private LoggedMechanismRoot2d driveRoot = viz2d.getRoot("drive", 0.25, 0);
  private LoggedMechanismLigament2d driveLigament =
      new LoggedMechanismLigament2d("drivetrain", 0.76, 0.0, 5.0, blue);

  private final double kElevatorPosX = 0.75;
  private final double kElevatorPosY = 0.51;
  private LoggedMechanismRoot2d elevatorRoot =
      viz2d.getRoot("elevatorRoot", kElevatorPosX, kElevatorPosY);

  private LoggedMechanismLigament2d elevatorLigament =
      new LoggedMechanismLigament2d("elevator", 0.13, 90.0, 20.0, white);

  private Pose3d elevatorBasePose3d = new Pose3d();
  private Pose3d elevatorMiddlePose3d = new Pose3d();
  private Pose3d elevatorTopPose3d = new Pose3d();
  private Color8Bit ledColor = new Color8Bit(0, 0, 0);

  public RobotVisualization(Elevator elevator) {
    this.elevator = elevator;
    if (RobotBase.isReal()) return;
    init2dViz();
  }

  private void init2dViz() {
    viz2d = new LoggedMechanism2d(1.27, 2.032);

    ledDisplayRoot = viz2d.getRoot("ledDisplay", 0.58, 1.5); // this goes above the robot
    ledDisplayBox =
        new LoggedMechanismLigament2d("LED State", 0.5, 0, 5.0, new Color8Bit(Color.kBlack));

    driveRoot = viz2d.getRoot("drive", 0.25, 0);
    driveLigament = new LoggedMechanismLigament2d("drivetrain", 0.76, 0.0, 5.0, blue);

    elevatorRoot = viz2d.getRoot("elevatorRoot", kElevatorPosX, kElevatorPosY);

    elevatorLigament = new LoggedMechanismLigament2d("elevator", 0.13, 90.0, 20.0, white);

    ledDisplayRoot.append(ledDisplayBox);
    driveRoot.append(this.driveLigament);

    // Set up elevator and rollers
    elevatorRoot.append(this.elevatorLigament);
  }

  public void updateViz() {
    // model_0 is elevator bottom cascade
    // model_1 is elevator middle cascade
    // model_2 is elevator top cascade
    double elevatorHeightM = this.elevator.getPosition().in(Meters);

    // Update elevator stages based on height
    double baseHeight = 0.0;
    double middleHeight = Math.max(0.0, elevatorHeightM - 0.631825);
    double topHeight = elevatorHeightM;

    elevatorBasePose3d = new Pose3d(0, 0, baseHeight, new Rotation3d(0, 0, 0));
    elevatorMiddlePose3d = new Pose3d(0, 0, middleHeight, new Rotation3d(0, 0, 0));
    elevatorTopPose3d = new Pose3d(0, 0, topHeight, new Rotation3d(0, 0, 0));

    Logger.recordOutput(
        "Visualization/ComponentsPoseArray",
        new Pose3d[] {elevatorBasePose3d, elevatorMiddlePose3d, elevatorTopPose3d});

    ledColor = LEDs.getInstance().getColor(0);

    if (RobotBase.isReal()) return;

    this.elevatorRoot.setPosition(kElevatorPosX, kElevatorPosY + elevatorHeightM);

    Logger.recordOutput("Visualization/Mechanisms2D", this.viz2d);

    ledDisplayBox.setColor(ledColor);
  }
}
