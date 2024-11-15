// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.configs;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ModuleConstants;

/** Add your docs here. */
public final class RevConfigs {
  public static final class MAXSwerveModule {
    public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      drivingConfig
          .idleMode(ModuleConstants.kDrivingMotorIdleMode)
          .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
      drivingConfig
          .encoder
          .positionConversionFactor(
              ModuleConstants.kDrivingEncoderPositionFactor.in(Meters)) // meters
          .velocityConversionFactor(
              ModuleConstants.kDrivingEncoderVelocityFactor.in(
                  MetersPerSecond)); // meters per second
      drivingConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
          .velocityFF(ModuleConstants.kDrivingFF)
          .outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);
      drivingConfig
          .signals
          .outputCurrentPeriodMs(10)
          .faultsPeriodMs(10)
          .motorTemperaturePeriodMs(20)
          .appliedOutputPeriodMs(10)
          .busVoltagePeriodMs(20)
          .primaryEncoderPositionPeriodMs(20)
          .primaryEncoderVelocityAlwaysOn(true)
          .primaryEncoderVelocityPeriodMs(20)
          .analogPositionPeriodMs(500)
          .analogVelocityPeriodMs(500)
          .analogVoltagePeriodMs(500)
          .absoluteEncoderPositionPeriodMs(500)
          .absoluteEncoderVelocityPeriodMs(500);

      turningConfig
          .idleMode(ModuleConstants.kTurningMotorIdleMode)
          .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
      turningConfig
          .absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(ModuleConstants.kTurningEncoderInverted)
          .positionConversionFactor(
              ModuleConstants.kTurningEncoderPositionFactor.in(Radians)) // radians
          .velocityConversionFactor(
              ModuleConstants.kTurningEncoderVelocityFactor.in(
                  RadiansPerSecond)); // radians per second
      turningConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD)
          .outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(
              ModuleConstants.kTurningEncoderPositionPIDMinInput.in(Radian),
              ModuleConstants.kTurningEncoderPositionPIDMaxInput.in(Radian));
      turningConfig
          .signals
          .outputCurrentPeriodMs(10)
          .faultsPeriodMs(10)
          .motorTemperaturePeriodMs(20)
          .appliedOutputPeriodMs(10)
          .busVoltagePeriodMs(20)
          .primaryEncoderPositionPeriodMs(20)
          .primaryEncoderVelocityPeriodMs(20)
          .analogPositionPeriodMs(500)
          .analogVelocityPeriodMs(500)
          .analogVoltagePeriodMs(500)
          .absoluteEncoderPositionPeriodMs(10)
          .absoluteEncoderVelocityPeriodMs(10);
    }
  }
}
