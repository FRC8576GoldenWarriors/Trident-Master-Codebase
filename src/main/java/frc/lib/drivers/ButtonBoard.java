package frc.lib.drivers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Subsystems.Vision.Limelight.LimelightConstants;
import frc.robot.Subsystems.Vision.TagMap;
import frc.robot.Subsystems.Vision.TagMap.Face;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class ButtonBoard implements Periodical {

  // counter clockwise from current alliance driver station
  private HashMap<FaceState, Supplier<Integer>> faceToTagMap =
      new HashMap<>(
          Map.of(
              FaceState.A, () -> this.isBlueAlliance() ? 18 : 7,
              FaceState.B, () -> this.isBlueAlliance() ? 17 : 8,
              FaceState.C, () -> this.isBlueAlliance() ? 22 : 9,
              FaceState.D, () -> this.isBlueAlliance() ? 21 : 10,
              FaceState.E, () -> this.isBlueAlliance() ? 20 : 11,
              FaceState.F, () -> this.isBlueAlliance() ? 19 : 6));

  public enum SideState {
    LEFT,
    RIGHT
  }

  public enum LevelState {
    L1,
    L2,
    L3,
    L4
  }

  public enum FaceState {
    A,
    B,
    C,
    D,
    E,
    F
  }

  private CommandGenericHID buttonBoard;
  private TagMap tagMap;

  private SideState currentSideState = SideState.RIGHT;
  private LevelState currentLevelState = LevelState.L1;
  private FaceState currentFaceState = FaceState.A;

  private double sideDistance = 0;
  private int tagID = -1;

  public ButtonBoard(CommandGenericHID buttonBoard, TagMap tagMap) {
    this.buttonBoard = buttonBoard;
    this.tagMap = tagMap;
    PeriodicalUtil.registerPeriodic(this);
  }

  public CommandGenericHID getGenericHID() {
    return buttonBoard;
  }

  public void setState(SideState sideState) {
    currentSideState = sideState;
  }

  public void setState(LevelState levelState) {
    currentLevelState = levelState;
  }

  public void setState(FaceState faceState) {
    currentFaceState = faceState;
  }

  private boolean isBlueAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
  }

  public DeferredCommand getCurrentCommand(Subsystem... requirements) {
    return tagMap.getReefAlignmentPathfindToPose(
        tagID,
        LimelightConstants.PhysicalConstants.DESIRED_APRIL_TAG_DISTANCE_REEF,
        sideDistance,
        Face.FrontSide,
        requirements);
  }

  // Could be used to clean up the bindings
  public void configureButtonBoardBindings() {
    // buttonBoard.button(1).onTrue(new InstantCommand(() -> this.setState(FaceState.A)));
  }

  @Override
  public void periodic() {

    sideDistance = 0;
    tagID = -1;

    switch (currentSideState) {
      case LEFT:
        sideDistance = LimelightConstants.PhysicalConstants.LEFT_STICK_OFFSET;
        break;
      case RIGHT:
        sideDistance = LimelightConstants.PhysicalConstants.RIGHT_STICK_OFFSET;
        break;
    }

    tagID = faceToTagMap.get(currentFaceState).get();
  }
}
