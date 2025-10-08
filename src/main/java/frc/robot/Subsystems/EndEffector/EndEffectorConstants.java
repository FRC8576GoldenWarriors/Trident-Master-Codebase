package frc.robot.Subsystems.EndEffector;

public class EndEffectorConstants {
  public class HardwareConstants {
    public static int sideRollerID = 10;
    public static boolean sideRollerInverted = false;//true; // In is positive
    public static int sideRollerCurrentLimit = 60;

    // public static int topRollerID = 12;
    // public static boolean topRollerInverted = false;
    // public static int topRollerCurrentLimit = 40;

    public static int pivotMotorID = 11;
    public static boolean pivotMotorInverted = true;
    public static int pivotMotorCurrentLimit = 60;

    public static int pivotThruBoreID = 3; // 9; // 1;
    public static double thruBoreExpectedZero = -0.41;
    public static double thruBoreFullRange = 1.0;
    public static boolean pivotEncoderInverted = true;

    public static int canRangeID = 1; // 2;
  }

  public class ControlConstants {
    public static double kP = 1000;//400; // 60;
    public static double kI = 0.0;
    public static double kD = 0.04;

    public static double kS = 0.0;
    public static double kG = 0.16; // 7;
    public static double kV = 0.0;
    public static double kA = 0.0;

    public static double groundIntakePosition = 0.02;//0;//0.05;
    public static double l1Position = 0.2;
    public static double l2Position = 0.2;
    public static double l3Position = 0.28;
    public static double l4Position = 0.42;//0.56//0.33;//Actual Value//0.47
    public static double l4BackPosition = 0.2;
    public static double holdPosition = 0.55;
    public static double COMOffset = 0.39446; // IN ROTATIONS
  }
}
