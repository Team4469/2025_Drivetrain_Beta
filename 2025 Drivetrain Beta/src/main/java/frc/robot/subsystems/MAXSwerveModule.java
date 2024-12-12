// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import frc.robot.configs.RevConfigs;

public class MAXSwerveModule {
  private final SparkFlex m_drivingSparkFlex;
  private final SparkMax m_turningSparkMax;

  private final SparkClosedLoopController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;

  private RelativeEncoder m_drivingEncoder;
  private AbsoluteEncoder m_turningEncoder;

  private Angle m_chassisAngularOffset = Radian.of(0);
  private SwerveModuleState m_desiredState =
      new SwerveModuleState(MetersPerSecond.of(0), new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor, encoder, and PID
   * controller. This configuration is specific to the REV MAXSwerve Module built with NEOs, SPARKS
   * MAX, and a Through Bore Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, Angle chassisAngularOffset) {
    // Motors
    m_drivingSparkFlex = new SparkFlex(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSparkFlex.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();

    // Closed Loop Controllers
    m_drivingPIDController = m_drivingSparkFlex.getClosedLoopController();
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();

    // Apply all configs to the motor controller. Always should be the last direct motor call
    m_drivingSparkFlex.configure(
        RevConfigs.MAXSwerveModule.drivingConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSparkMax.configure(
        RevConfigs.MAXSwerveModule.turningConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Chassis Configuration
    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());

    m_drivingEncoder.setPosition(0);
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
        new Rotation2d(Radian.of(m_turningEncoder.getPosition()).minus(m_chassisAngularOffset)));
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
        Meters.of(m_drivingEncoder.getPosition()),
        new Rotation2d(Radian.of(m_turningEncoder.getPosition()).minus(m_chassisAngularOffset)));
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
        desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset.in(Radian)));

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
