package frc.robot.Subsystems.EndEffector;

public class EndEffectorConstants {
  public class HardwareConstants {
    public static int sideRollerID = 10;
    public static boolean sideRollerInverted = false;
    public static int sideRollerCurrentLimit = 60;

    public static int topRollerID = 12;
    public static boolean topRollerInverted = false;
    public static int topRollerCurrentLimit = 40;

    public static int pivotMotorID =  11;
    public static boolean pivotMotorInverted = true;
    public static int pivotMotorCurrentLimit = 60;

    public static int pivotThruBoreID = 9; // 1;
    public static double thruBoreExpectedZero = 0.27;
    public static double thruBoreFullRange = 1.0;

    public static int canRangeID = 1; // 2;
  }

  public class ControlConstants {
    public static double kP = 60;
    public static double kI = 0.0;
    public static double kD = 0.0;

    
    public static double kS = 0.0;
    public static double kG = 7;
    public static double kV = 0.0;
    public static double kA = 0.0;

    
    public static double groundIntakePosition = 0.1;
    public static double l4Position = 0.24;
    public static double COMOffset = 0.39446;//IN ROTATIONS
  }
}
