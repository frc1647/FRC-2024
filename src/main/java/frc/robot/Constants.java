// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    // Port numbers for driver and operator gamepads. These correspond with the numbers on the USB
    // tab of the DriverStation
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class DrivetrainConstants {
    // PWM ports/CAN IDs for motor controllers
    public static final int kLeftFrontID = 1;
    public static final int kLeftRearID = 2;
    public static final int kRightFrontID = 3;
    public static final int kRightRearID = 4;

    public static final double kRampRate = .5;

    public static final double kDriveDeadband = .05;

    public static final double kPositionConversionFactor = 0.04545;
    public static final double kVelocityConversionFactor = 0.00061599855;//(1.0/12.75) * 2.0 * Math.PI * .075 / 60.0;

    ; // may change or be unnessary

    public static final double kDrivetrainWidthMeters = .5; //temporary
  }

  public static class LauncherConstants {
    // PWM ports/CAN IDs for motor controllers
    public static final int kFeederID = 5;
    public static final int kLauncherID = 6;

    // Current limit for launcher and feed wheels
    public static final int kLauncherCurrentLimit = 80;
    public static final int kFeedCurrentLimit = 80;

    // Speeds for wheels when intaking and launching. Intake speeds are negative to run the wheels
    // in reverse
    public static final double kLauncherSpeed = 1;
    public static final double kLaunchFeederSpeed = 1;
    public static final double kIntakeLauncherSpeed = -.5;
    public static final double kIntakeFeederSpeed = -.2;

    public static final double kLauncherDelay = 1;
  }

  public static class IntakeConstants {
    public static final double kIntakeSpeed = 0.3;
    public static final int kArmID = 13; //Falcon 500 (Talon FX)
    public static final int kRollerID = 12; //Talon SPX
    public static final double kRollerIntakeVolts = 10;
  }

  public static class ClimberConstants {
    public static final int kLeftClimberID = 16;
    public static final int kRightClimberID = 15;

    public static final double kDeadzone = 0.03;
    
    public static final double kLeftDownSpeed = 1; //temporary
    public static final double kRightDownSpeed = 1; //temporary
    public static final double kLeftUpSpeed = 1; //temporary
    public static final double kRightUpSpeed = 1.00; //temporary
  }
  public static class CameraConstants {

    // height of apriltag - height of camera
    public static final double kApriltagHeightDifferenceInches = 57 - 6;
    public static final double kCameraAngle = 30;
  }

  public static class AutoConstants {

    public static class RamseteConstants {
      public static final double kRamseteOutputMPS = 1.0; //might work

      public static final double ksVolts = 0.2244;
      public static final double kvVoltSecondsPerMeter = 3.4247;
      public static final double kaVoltSecondsSquaredPerMeter = 0.62693;

      // PID values (temporary)
      public static final double kPDriveVel = 0.05;
    }

    public static class lookAtApriltagConstants {
    // height of apriltag - height of camera
      public static final double kP = .03;
      //public static final double kI = 34;
    }
  }
  public static class sysIdLogConstants{
    public static final boolean kForward = true;
  }
}
