// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static class TalonConstants {
    public static final int talonFXTicks = 2048;
    public static final int talonSRXTicks = 4096;

    public static final double MAX_VOLTAGE = 10.0;

    public static final int kPIDIdx = 0;
    public static final int kTimeoutMs = 10;
    public static final boolean kIsPracticeBot = false;
    public static final double kVoltageComp = 10.0;
    public static final double kCurrentLimit = 35;
    public static final double kDriveCurrentLimit = 40;
    public static final double kSteerCurrentLimit = 60;
  }

  public static class VisionConstants {
        //Camera name
        public static final String cameraName = "monocle";

        //Position of camera relative to robot center (meters, radians)
        public static final Transform3d camToRobot = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
                
        
  }

  public static class SwerveConstants {

      public static final double headingThreshold = 0.5; //heading correction stops if its off by < 5 degrees
      public static final double joystickThreshold = 0.05;
      public static final double headingTimerThreshold = 1;

      // The steer motor uses any SwerveModule.SteerRequestType control request with the
      // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
      private static final Slot0Configs steerGains = new Slot0Configs()
          .withKP(100).withKI(0).withKD(0.05)
          .withKS(0).withKV(1.5).withKA(0);

      // When using closed-loop control, the drive motor uses the control
      // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
      private static final Slot0Configs driveGains = new Slot0Configs()
          .withKP(3).withKI(0).withKD(0)
          .withKS(0).withKV(0).withKA(0);

      // The closed-loop output type to use for the steer motors;
      // This affects the PID/FF gains for the steer motors
      private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
      
      // The closed-loop output type to use for the drive motors;
      // This affects the PID/FF gains for the drive motors
      private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

      // The stator current at which the wheels start to slip;
      // This needs to be tuned to your individual robot
      private static final double kSlipCurrentA = 300.0;

      // Theoretical free speed (m/s) at 12v applied output;
      // This needs to be tuned to your individual robot
      public static final double kSpeedAt12VoltsMps = 5.5; //Uhh the higher the better, do not do <5

      // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
      // This may need to be tuned to your individual robot
      private static final double kCoupleRatio = 3.5;

      private static final double kDriveGearRatio = 7.363636364;
      private static final double kSteerGearRatio = 15.42857143;
      private static final double kWheelRadiusInches = 2.05; // Estimated at first, then fudge-factored to make odom match record

      private static final boolean kSteerMotorReversed = true;
      private static final boolean kInvertLeftSide = false;
      private static final boolean kInvertRightSide = false;

      private static final String kCANbusName = "rio";

      // These are only used for simulation
      private static final double kSteerInertia = 0.00001;
      private static final double kDriveInertia = 0.001;

      //Pigeon ID
      public static final int kPigeonId = 0;
      public static boolean usingPigeon = true;
      
      // Simulated voltage necessary to overcome friction
      private static final double kSteerFrictionVoltage = 0.25;
      private static final double kDriveFrictionVoltage = 0.25;

      static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
              .withCANbusName(kCANbusName)
              .withPigeon2Id(kPigeonId); //(if using pigeon)

      private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
              .withDriveMotorGearRatio(kDriveGearRatio)
              .withSteerMotorGearRatio(kSteerGearRatio)
              .withWheelRadius(kWheelRadiusInches)
              .withSlipCurrent(kSlipCurrentA)
              .withSteerMotorGains(steerGains)
              .withDriveMotorGains(driveGains)
              .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
              .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
              .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
              .withSteerInertia(kSteerInertia)
              .withDriveInertia(kDriveInertia)
              .withSteerFrictionVoltage(kSteerFrictionVoltage)
              .withDriveFrictionVoltage(kDriveFrictionVoltage)
              //.withFeedbackSource(SteerFeedbackType.FusedCANcoder)
              .withCouplingGearRatio(kCoupleRatio)
              .withSteerMotorInverted(kSteerMotorReversed);

      // Front Left
      public static final int kFrontLeftDriveMotorId = 14;
      public static final int kFrontLeftSteerMotorId = 2;
      public static final int kFrontLeftEncoderId = 9;
      public static final double kFrontLeftEncoderOffset = 0.40234375;
  
      private static final double kFrontLeftXPosInches = 13;
      private static final double kFrontLeftYPosInches = 13;

      // Front Right
      public static final int kFrontRightDriveMotorId = 49;
      public static final int kFrontRightSteerMotorId = 27;
      public static final int kFrontRightEncoderId = 11;
      public static final double kFrontRightEncoderOffset = 0.376220703125;
      
      private static final double kFrontRightXPosInches = 13;
      private static final double kFrontRightYPosInches = -13;

      // Back Left
      private static final int kBackLeftDriveMotorId = 39;
      private static final int kBackLeftSteerMotorId = 50;
      private static final int kBackLeftEncoderId = 10;
      public static final double kBackLeftEncoderOffset = -0.135498046875;
      
      private static final double kBackLeftXPosInches = -13;
      private static final double kBackLeftYPosInches = 13;

      // Back Right
      private static final int kBackRightDriveMotorId = 1;
      private static final int kBackRightSteerMotorId = 8;
      private static final int kBackRightEncoderId = 12;
      public static final double kBackRightEncoderOffset = 0.233154296875;
  
      private static final double kBackRightXPosInches = -13;
      private static final double kBackRightYPosInches = -13;

      public static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
              kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
      public static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
              kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
      public static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
              kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
      public static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
              kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);  
  }
}