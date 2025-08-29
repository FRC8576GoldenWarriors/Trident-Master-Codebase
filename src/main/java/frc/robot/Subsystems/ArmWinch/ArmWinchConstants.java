package frc.robot.Subsystems.ArmWinch;

public class ArmWinchConstants {
  public class HardwareConstants {
    public static int pivotSparkMaxID = 21;
    public static int encoderPort1 = 1;
    public static int encoderPort2 = 2;
    public static boolean motorInverted = true;
    public static boolean thruBoreInverted = false;
    public static double thruBoreExpectedZero = 0.0;
    public static double thruBoreFullRange = 1.0;
    public static int currentLimit = 25;
  }

  public class ControlConstants {
    public static double kP = 0.4;
    public static double kI = 0.0;
    public static double kD = 0.01;
  }
}
