// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
  private final SparkFlex m_drivingSparkFlex;
  private final SparkMax m_turningSparkMax;

  private final SparkClosedLoopController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;

  private RelativeEncoder m_drivingEncoder;
  private AbsoluteEncoder m_turningEncoder;

  private SparkFlexConfig m_driveConfig;
  private SparkMaxConfig m_turningConfig;

  private SignalsConfig m_driveSignalsConfig;
  private SignalsConfig m_turningSignalsConfig;

  private EncoderConfig m_driveEncoderConfig;
  private AbsoluteEncoderConfig m_turningEncoderConfig;

  private ClosedLoopConfig m_drivingClosedLoopConfig;
  private ClosedLoopConfig m_turningClosedLoopConfig;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor, encoder, and PID
   * controller. This configuration is specific to the REV MAXSwerve Module built with NEOs, SPARKS
   * MAX, and a Through Bore Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    // Driving Motor
    m_drivingSparkFlex = new SparkFlex(drivingCANId, MotorType.kBrushless);

    // Encoder Configs
    m_driveEncoderConfig.positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    m_driveEncoderConfig.velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
    
    // Closed Loop Configs
    m_drivingPIDController = m_drivingSparkFlex.getClosedLoopController();

    m_drivingClosedLoopConfig.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
    m_drivingClosedLoopConfig.outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);
    m_drivingClosedLoopConfig.pidf(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD, ModuleConstants.kDrivingFF);

    // Signal Configs
    m_driveSignalsConfig.outputCurrentPeriodMs(10);
    m_driveSignalsConfig.faultsPeriodMs(10);
    m_driveSignalsConfig.motorTemperaturePeriodMs(20);
    m_driveSignalsConfig.appliedOutputPeriodMs(10);
    m_driveSignalsConfig.busVoltagePeriodMs(20);
    m_driveSignalsConfig.primaryEncoderPositionPeriodMs(20);
    m_driveSignalsConfig.primaryEncoderVelocityAlwaysOn(true);
    m_driveSignalsConfig.primaryEncoderVelocityPeriodMs(20);
    m_driveSignalsConfig.analogPositionPeriodMs(500);
    m_driveSignalsConfig.analogVelocityPeriodMs(500);
    m_driveSignalsConfig.analogVoltagePeriodMs(500);
    m_driveSignalsConfig.absoluteEncoderPositionPeriodMs(500);
    m_driveSignalsConfig.absoluteEncoderVelocityPeriodMs(500);

    // Motor Configs
    m_driveConfig.idleMode(ModuleConstants.kDrivingMotorIdleMode);
    m_driveConfig.smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);

    m_driveConfig.apply(m_driveEncoderConfig);
    m_driveConfig.apply(m_drivingClosedLoopConfig);
    m_driveConfig.apply(m_driveSignalsConfig);


    // Turning Motor
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

    // Encoder Configs
    m_turningEncoderConfig.positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    m_turningEncoderConfig.velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
    m_turningEncoderConfig.inverted(ModuleConstants.kTurningEncoderInverted);

    // Closed Loop Configs
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();
    m_turningClosedLoopConfig.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    m_turningClosedLoopConfig.positionWrappingEnabled(true);
    m_turningClosedLoopConfig.positionWrappingInputRange(ModuleConstants.kTurningEncoderPositionPIDMinInput, ModuleConstants.kTurningEncoderPositionPIDMaxInput);
    m_turningClosedLoopConfig.pidf(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD, ModuleConstants.kTurningFF);
    m_turningClosedLoopConfig.outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);

    // Signal Configs
    m_turningSignalsConfig.outputCurrentPeriodMs(10);
    m_turningSignalsConfig.faultsPeriodMs(10);
    m_turningSignalsConfig.motorTemperaturePeriodMs(20);
    m_turningSignalsConfig.appliedOutputPeriodMs(10);
    m_turningSignalsConfig.busVoltagePeriodMs(20);
    m_turningSignalsConfig.primaryEncoderPositionPeriodMs(20);
    m_turningSignalsConfig.primaryEncoderVelocityPeriodMs(20);
    m_turningSignalsConfig.analogPositionPeriodMs(500);
    m_turningSignalsConfig.analogVelocityPeriodMs(500);
    m_turningSignalsConfig.analogVoltagePeriodMs(500);
    m_turningSignalsConfig.absoluteEncoderPositionPeriodMs(10);
    m_turningSignalsConfig.absoluteEncoderVelocityPeriodMs(10);

    // Motor Configs
    m_turningConfig.smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
    m_turningConfig.idleMode(ModuleConstants.kTurningMotorIdleMode);

    m_turningConfig.apply(m_turningEncoderConfig);
    m_turningConfig.apply(m_turningClosedLoopConfig);
    m_turningConfig.apply(m_turningSignalsConfig);


    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());


    m_drivingEncoder.setPosition(0);

    // Apply all configs to the motor controller. Always should be the last direct motor call
    m_drivingSparkFlex.configure(m_driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningSparkMax.configure(m_turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
        m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle =
        desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(
        correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningPIDController.setReference(
        correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public boolean getDrivingCanRxFault() {
    return m_drivingSparkFlex.getFaults().can;
  }

  public boolean getDrivingCanTxFault() {
    return m_drivingSparkFlex.getFaults().can;
  }

  public double getDrivingCurrent() {
    return m_drivingSparkFlex.getOutputCurrent();
  }

  public double getDrivingOutput() {
    return m_drivingSparkFlex.getAppliedOutput();
  }

  public double getBusVoltage() {
    return m_drivingSparkFlex.getBusVoltage();
  }
}
