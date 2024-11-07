// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.*;

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
    public static final int kDriverControllerPort = 0;
  }

   public static final class DriveConstants {

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 5.7;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kMaxAngularAcceleration =
        kMaxAngularSpeed * 1.5; // FIND Actual number

    public static final double kDirectionSlewRate = 2; // radians per second
    public static final double kMagnitudeSlewRate = 2.2; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.5; // percent per second (1 = 100%)

    // Chassis configuration
    public static final Distance kModuleInset = Inches.of(1.75); //Units.inchesToMeters(1.75);
    public static final Distance kRobotChassisLength = Inches.of(27);
    public static final Distance kRobotChassisWidth = Inches.of(27);
    public static final Distance kTrackWidth = kRobotChassisLength.minus(kModuleInset.times(2));
        //kRobotChassisLengthMeters - (2 * kModuleInsetMeters);
    // Distance between centers of right and left wheels on robot
    public static final Distance kWheelBase = kRobotChassisWidth.minus(kModuleInset.times(2));
        //kRobotChassisWidthMeters - (2 * kModuleInsetMeters);
    // Distance between front and back wheels on robot
    public static final Distance kRobotDriveRadius = Inches.of(Math.abs(Math.hypot(kTrackWidth.in(Inches), kWheelBase.in(Inches))));
        // Math.abs(Math.hypot(kTrackWidthMeters / 2, kWheelBaseMeters / 2));
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase.divide(2), kTrackWidth.divide(2)),
            new Translation2d(kWheelBase.divide(2), kTrackWidth.divide(2).unaryMinus()),
            new Translation2d(kWheelBase.divide(2).unaryMinus(), kTrackWidth.divide(2)),
            new Translation2d(kWheelBase.divide(2).unaryMinus(), kTrackWidth.divide(2)).unaryMinus());

    // Angular offsets of the modules relative to the chassis in radians
    public static final Angle kFrontLeftChassisAngularOffset = Radians.of(-Math.PI / 2);
    public static final Angle kFrontRightChassisAngularOffset = Radians.of(0);
    public static final Angle kBackLeftChassisAngularOffset = Radians.of(Math.PI);
    public static final Angle kBackRightChassisAngularOffset = Radians.of(Math.PI / 2);

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 10;
    public static final int kFrontLeftTurningCanId = 11;

    public static final int kFrontRightDrivingCanId = 56;
    public static final int kFrontRightTurningCanId = 57;

    public static final int kRearRightDrivingCanId = 14;
    public static final int kRearRightTurningCanId = 15;

    public static final int kRearLeftDrivingCanId = 16;
    public static final int kRearLeftTurningCanId = 17;

    public static final boolean kGyroReversed = true;
  }
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14; // High Speed

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
    // bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22.0) / (kDrivingMotorPinionTeeth * 15.0);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor =
        (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor =
        ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor =
        (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput =
        kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }
}
