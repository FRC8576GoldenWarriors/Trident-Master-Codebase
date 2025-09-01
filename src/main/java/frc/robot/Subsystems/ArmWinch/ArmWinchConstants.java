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
    public static int currentLimit = 35;//25;
  }

  public class ControlConstants {
    public static double kP = 0.8;//0.6; // 0.8;//0.5;//0.45;
    public static double kI = 0.0;
    public static double kD = 0.01; // 0.01;//0.01;//0.03;//0.02;

    public static double kG = 0.1;

    public static double maxVelocity = 1040; // 540;//360;//240;//180;
    public static double maxAcceleration = 1400; // 720;//540;//360;//240;
  }
}
