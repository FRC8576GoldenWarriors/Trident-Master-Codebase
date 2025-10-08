// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.drivers.ButtonBoard.SideState;
import frc.robot.Subsystems.SwerveDrive.Drivetrain;
import frc.robot.Subsystems.SwerveDrive.SwerveConstants;
import frc.robot.Subsystems.Vision.Limelight.Limelight;
import frc.robot.Subsystems.Vision.Limelight.LimelightConstants;
import frc.robot.Subsystems.Vision.TagMap;
import frc.robot.Subsystems.Vision.TagMap.Face;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefAlign extends Command {

  private Limelight limelight;
  private Drivetrain drivetrain;
  private TagMap tagMap;
  private String name;

  private final ProfiledPIDController rotationPID;
  private final PIDController forwardPID;
  private final PIDController strafePID;

  private double driveOutput;
  private double rotationOutput;
  private double strafeOutput;

  private HashMap<String, Supplier<Pair<Double, Double>>> limelightNameToDistanceAndAngle =
      new HashMap<>();

  public ReefAlign(Limelight limelight, Drivetrain drivetrain, TagMap tagMap, SideState sideState) {

    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.tagMap = tagMap;

    rotationPID =
        new ProfiledPIDController(
            LimelightConstants.PIDConstants.rotationkP,
            LimelightConstants.PIDConstants.rotationkI,
            LimelightConstants.PIDConstants.rotationkD,
            new Constraints(
                SwerveConstants.DRIVETRAIN_MAX_ANGULAR_SPEED,
                SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION));
    rotationPID.setTolerance(LimelightConstants.PIDConstants.ALLOWED_ANGLE_ERROR);
    rotationPID.enableContinuousInput(-180, 180);

    forwardPID =
        new PIDController(
            LimelightConstants.PIDConstants.forwardkP,
            LimelightConstants.PIDConstants.forwardkI,
            LimelightConstants.PIDConstants.forwardkD);
    forwardPID.setTolerance(LimelightConstants.PIDConstants.ALLOWED_DISTANCE_ERROR);
    forwardPID.setSetpoint(LimelightConstants.PhysicalConstants.DESIRED_APRIL_TAG_DISTANCE_REEF);

    strafePID =
        new PIDController(
            LimelightConstants.PIDConstants.strafekP,
            LimelightConstants.PIDConstants.strafekI,
            LimelightConstants.PIDConstants.strafekD);
    strafePID.setTolerance(LimelightConstants.PIDConstants.ALLOWED_STRAFE_ERROR);

    switch (sideState) {
      case LEFT:
        strafePID.setSetpoint(
            LimelightConstants.PhysicalConstants.LEFT_STICK_OFFSET); // set these values correctly
        this.name = LimelightConstants.NameConstants.LEFT_NETWORKTABLE_KEY;
        break;

      case RIGHT:
        strafePID.setSetpoint(
            LimelightConstants.PhysicalConstants.RIGHT_STICK_OFFSET); // set these values correctly
        this.name = LimelightConstants.NameConstants.RIGHT_NETWORKTABLE_KEY;
        break;
    }

    LimelightConstants.NameConstants.LimelightKeys.forEach(
        (name) ->
            limelightNameToDistanceAndAngle.put(
                name,
                () -> Pair.of(limelight.getDistanceToTagUsingTrig(name), limelight.getYaw(name))));

    addRequirements(limelight, drivetrain);
  }

  @Override
  public void initialize() {
    forwardPID.reset();
    rotationPID.reset(drivetrain.getHeading(), drivetrain.getRotationVelocity());
    strafePID.reset();
  }

  @Override
  public void execute() {

    List<Boolean> hasTargets =
        LimelightConstants.NameConstants.LimelightKeys.stream()
            .map((name) -> limelight.hasTargets(name))
            .toList();

    if (!hasTargets.contains(true)) return;

    double sideDistance = 0;
    double averageDistance = 0;

    var values = limelightNameToDistanceAndAngle.values();

    for (Supplier<Pair<Double, Double>> distanceAnglePair : values) {
      Pair<Double, Double> pair = distanceAnglePair.get();
      averageDistance += pair.getFirst() * Math.sin(Math.toRadians(pair.getSecond()));
    }

    averageDistance /= values.size();

    var sideValues = limelightNameToDistanceAndAngle.get(name).get();

    sideDistance = sideValues.getFirst() * Math.cos(Math.toRadians(sideValues.getSecond()));

    driveOutput = forwardPID.calculate(averageDistance);
    strafeOutput = strafePID.calculate(sideDistance);
    rotationOutput =
        rotationPID.calculate(
            drivetrain.getHeading(),
            tagMap.getAlignRotationInDegrees(limelight.getTagID(name), Face.FrontSide));

    drivetrain.drive(new Translation2d(driveOutput, strafeOutput), rotationOutput, false, true);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
