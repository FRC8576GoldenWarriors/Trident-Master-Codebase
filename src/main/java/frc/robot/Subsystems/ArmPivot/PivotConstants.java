package frc.robot.Subsystems.ArmPivot;

public class PivotConstants {
  public class HardwareConstants {
    public static int pivotSparkMaxID = 22;
    public static int thruBoreID = 0; // 7;//6;//4;
    public static boolean motorInverted = true;
    public static boolean thruBoreInverted = true;
    public static double thruBoreExpectedZero = -0.04;
    public static double thruBoreFullRange = 1.0;
    public static int currentLimit = 25; // 20;
  }

  public class ControlConstants {
    public static double kP = 275;//320;//0.08;//0;//160; // 128;//256;//512.0; // 256.0
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double kS = 0.0; // 0.02;
    public static double kV = 0.0;
    public static double kG = 0.2;//0;//0.4;
    public static double kA = 0.0;

    public static double frontL1 = 0.1;//0.07;
    public static double frontL2 = 0.1;
    public static double frontL3 = 0.15;
    public static double frontL4 = 0.2;

    public static double backL1 = 0.0;
    public static double backL2 = 0.0;
    public static double backL3 = 0.0;
    public static double backL4 = 0.0;

    public static double groundIntake = 0.04;
    public static double stationIntake = 0.0;

    public static double setClimb = 0.0;
    public static double climbUp = 0.0;
    public static double climbDown = 0.0;

    public static double startPosition = 0.0;
    public static double COMOffset = 0.3976; // IN ROTATIONS
  }
}
