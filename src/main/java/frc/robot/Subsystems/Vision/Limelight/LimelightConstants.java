package frc.robot.Subsystems.Vision.Limelight;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class LimelightConstants {

  public static class NameConstants {
    public static final String RIGHT_NETWORKTABLE_KEY = "limelight-barge";
    public static final String LEFT_NETWORKTABLE_KEY = "limelight-reef";
    public static final List<String> LimelightKeys =
        Arrays.asList(RIGHT_NETWORKTABLE_KEY, LEFT_NETWORKTABLE_KEY);
  }

  public static class PositionalConstants {
    public static final Pose3d RIGHT_LIMELIGHT_LOCATION =
        new Pose3d(
            new Translation3d(
                -Units.inchesToMeters(3.680140),
                -Units.inchesToMeters(8.282677),
                Units.inchesToMeters(9.662383)),
            new Rotation3d(
                Units.degreesToRadians(0), Units.degreesToRadians(15), Units.degreesToRadians(0)));

    public static final Pose3d LEFT_LIMELIGHT_LOCATION =
        new Pose3d(
            new Translation3d(
                -Units.inchesToMeters(3.684619),
                Units.inchesToMeters(7.982677),
                Units.inchesToMeters(9.679096)),
            new Rotation3d(
                Units.degreesToRadians(0), Units.degreesToRadians(15), Units.degreesToRadians(0)));
  }

  public static class PhysicalConstants {
    public static final double FOCAL_LENGTH = 4.1;
    public static final double REAL_WIDTH = 165.0;
    public static final double PIXEL_WIDTH = 320.0;
    public static final double DESIRED_APRIL_TAG_DISTANCE_BARGE = 2.75;
    public static final double DESIRED_APRIL_TAG_DISTANCE_BARGE_REDALGAE = 1.85; // 1.9;
    public static final double DESIRED_APRIL_TAG_DISTANCE_REEF = 0.125;
    public static final double LEFT_STICK_OFFSET = -0.3;
    public static final double RIGHT_STICK_OFFSET = 0.3;

    public static final HashMap<Integer, Double> tagMap =
        new HashMap<>(
            Map.ofEntries(
                Map.entry(18, 180.0),
                Map.entry(7, 180.0),
                Map.entry(17, -120.0),
                Map.entry(6, 120.0),
                Map.entry(22, -60.0),
                Map.entry(11, 60.0),
                Map.entry(21, 0.0),
                Map.entry(10, 0.0),
                Map.entry(20, 60.0),
                Map.entry(9, -60.0),
                Map.entry(19, 120.0),
                Map.entry(8, -120.0)));
  }

  public static class PoseEstimationConstants {
    public static final boolean useDynamicVisionDeviations = true;

    public static final double baseDrivetrainXDeviaition = 0.01;
    public static final double baseDrivetrainYDeviaition = 0.01;
    public static final double baseDrivetrainThetaDeviation = 0.0000001;

    public static final double baseVisionXDeviaition = 0.3;
    public static final double baseVisionYDeviaition = 0.3;
    public static final double baseVisionThetaDeviaition = 9999999;
  }

  public static class PIDConstants {
    public static final double rotationkP = 0.08;
    public static final double rotationkI = 0.00;
    public static final double rotationkD = 0.001;
    public static final double ALLOWED_ANGLE_ERROR = 4.0;

    public static final double forwardkP = 2.3;
    public static final double forwardkI = 0.04;
    public static final double forwardkD = 0.001;
    public static final double ALLOWED_DISTANCE_ERROR = 0.5;

    public static final double strafekP = 2.3; // 0.1;
    public static final double strafekI = 0.0;
    public static final double strafekD = 0.001;
    public static final double ALLOWED_STRAFE_ERROR = 0.016; // 1.0;
  }

  public static enum reefOffsets {
    LEFT,
    CENTER,
    RIGHT
  }
}
