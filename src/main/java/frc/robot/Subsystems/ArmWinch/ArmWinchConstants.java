package frc.robot.Subsystems.ArmWinch;

public class ArmWinchConstants {
  public class HardwareConstants {
    public static int pivotSparkMaxID = 21;
    public static int encoderPort1 = 1;
    public static int encoderPort2 = 2;
    public static boolean motorInverted = false;
    public static boolean thruBoreInverted = false;
    public static double thruBoreExpectedZero = 0.0;
    public static double thruBoreFullRange = 1.0;
    public static int currentLimit = 35; // 25;
  }

  public class ControlConstants {
    public static double kP =
        .64; // Start at .64. .32 and lower tested w/ threshold//0.0; // 16384;//8192.0; // 0.6; //
    // 0.8;//0.5;//0.45;
    public static double kI = 0.0;
    public static double kD = 0; // 0.01; // 0.01;//0.01;//0.03;//0.02;

    public static double kS = 0.0;
    public static double kG = 0.15; // 0.0; // 0.1;
    public static double kV = 0.0; // 0.1;
    public static double kA = 0.0; // 0.1;

    public static double maxVelocity = 8000; // 4000; // 540;//360;//240;//180;
    public static double maxAcceleration = 10000; // 3000; // 720;//540;//360;//240;

    public static double holding = 400;
    public static double frontL1 = 0;//2000;
    public static double frontL2 = 0;//2500;
    public static double frontL3 = 0;//4300;
    public static double frontL4 = 1200;//6000;

    public static double backL1 = 0.0;
    public static double backL2 = 0.0;
    public static double backL3 = 0.0;
    public static double backL4 = 0.0;

    public static double groundIntake = 0.00;
    public static double stationIntake = 0.0;

    public static double testPosition = 1200;
  }
}
