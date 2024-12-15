// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Newtons;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification.NotificationLevel;

import java.util.Optional;

public class DriveSubsystem extends SubsystemBase {
  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;

  private final Field2d m_field = new Field2d();

  private RobotConfig robotConfig;

  private double[] feedforwardVoltage;

  private boolean mt2FrontConnected;

  boolean doRejectUpdate;

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft =
      new MAXSwerveModule(
          DriveConstants.kFrontLeftDrivingCanId,
          DriveConstants.kFrontLeftTurningCanId,
          DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight =
      new MAXSwerveModule(
          DriveConstants.kFrontRightDrivingCanId,
          DriveConstants.kFrontRightTurningCanId,
          DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft =
      new MAXSwerveModule(
          DriveConstants.kRearLeftDrivingCanId,
          DriveConstants.kRearLeftTurningCanId,
          DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight =
      new MAXSwerveModule(
          DriveConstants.kRearRightDrivingCanId,
          DriveConstants.kRearRightTurningCanId,
          DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  // Odometry class for tracking robot pose

  SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics, getHeading(), getModulePositions(), new Pose2d());

  /** Creates a new DriveSubsystem. */
  @SuppressWarnings("unused")
  public DriveSubsystem() {
    Shuffleboard.getTab("Field")
        .add("Field", m_field)
        .withWidget(BuiltInWidgets.kField)
        .withPosition(4, 0)
        .withSize(7, 5);
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file

    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    setpointGenerator =
        new SwerveSetpointGenerator(robotConfig, Constants.DriveConstants.kMaxAngularSpeed);

    ChassisSpeeds currentSpeeds = getRobotRelativeSpeeds();
    SwerveModuleState[] currentStates = getModuleStates();
    previousSetpoint =
        new SwerveSetpoint(
            currentSpeeds, currentStates, DriveFeedforwards.zeros(robotConfig.numModules));

    mt2FrontConnected = true;

    final boolean USE_FEEDFORWARD_FORCES = true;
    final boolean USE_SETPOINT_GENERATOR = false;

    if (USE_FEEDFORWARD_FORCES && USE_SETPOINT_GENERATOR) {
      System.out.println(
          "WARNING DRIVE: Both advanced options are enabled, will default to setpoint generator");
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> {
          if ((USE_FEEDFORWARD_FORCES && USE_SETPOINT_GENERATOR)) {
            driveRobotRelative(speeds);
          } else if (USE_SETPOINT_GENERATOR) {
            driveRobotRelative(speeds);
          } else if (USE_FEEDFORWARD_FORCES) {
            driveAuto(speeds, feedforwards.linearForces());
          } else {
            drive(speeds, false);
          }
        },
        // Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following
            // controller for holonomic drive trains
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
        robotConfig, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    };
  }

  // COMMANDS
  public Command setXCommand() {
    return this.runOnce(this::setX);
  }

  public Command zeroGyro() {
    return this.runOnce(this::zeroHeading);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeed.in(MetersPerSecond);
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeed.in(MetersPerSecond);
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed.in(RadiansPerSecond);

    // get alliance color
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent() && ally.get() == Alliance.Red) {
      xSpeedDelivered *= -1;
      ySpeedDelivered *= -1;
    }

    drive(new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered), fieldRelative);
  }

  private void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    if (fieldRelative)
      speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              speeds,
              getPose()
                  .getRotation()); // Convert from field relative to robot relative for determining
    // speeds
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeed);
    setModuleStates(swerveModuleStates);
  }

  private void driveAuto(ChassisSpeeds speeds, Force[] feedforwardForces) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeed);

    // necessary information to calculate feed-forward. Same for all modules in our current config
    DCMotor driveMotorModel = DCMotor.getNeoVortex(1);
    double driveGearRatio = Constants.ModuleConstants.kDrivingMotorReduction;
    Distance wheelRadius = Constants.ModuleConstants.kWheelDiameter.divide(2); // YAGSL in meters

    for (int i = 0; i < 4; i++) {
      // calculation:
      double desiredGroundSpeedMPS = swerveModuleStates[i].speedMetersPerSecond;
      feedforwardVoltage[i] =
          driveMotorModel.getVoltage(
              // Since: (1) torque = force * momentOfForce; (2) torque (on wheel) = torque (on
              // motor) * gearRatio
              // torque (on motor) = force * wheelRadius / gearRatio
              feedforwardForces[i].in(Newtons) * wheelRadius.in(Meter) / driveGearRatio,
              // Since: (1) linear velocity = angularVelocity * wheelRadius; (2) wheelVelocity =
              // motorVelocity / gearRatio
              // motorAngularVelocity = linearVelocity / wheelRadius * gearRatio
              desiredGroundSpeedMPS / wheelRadius.in(Meter) * driveGearRatio);
    }

    setModuleStatesAuto(swerveModuleStates, feedforwardVoltage);
  }

  /**
   * This method will take in desired robot-relative chassis speeds, generate a swerve setpoint,
   * then set the target state for each module
   *
   * @param speeds The desired robot-relative speeds
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    // Note: it is important to not discretize speeds before or after
    // using the setpoint generator, as it will discretize them for you
    previousSetpoint =
        setpointGenerator.generateSetpoint(
            previousSetpoint, // The previous setpoint
            speeds, // The desired target speeds
            0.02 // The loop time of the robot code, in seconds
            );
    setModuleStates(
        previousSetpoint
            .moduleStates()); // Method that will drive the robot given target module states
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void setModuleStatesAuto(SwerveModuleState[] desiredStates, double[] feedforwardVoltages) {
    m_frontLeft.setDesiredStateWithFeedForward(desiredStates[0], feedforwardVoltages[0]);
    m_frontRight.setDesiredStateWithFeedForward(desiredStates[0], feedforwardVoltages[0]);
    m_rearLeft.setDesiredStateWithFeedForward(desiredStates[0], feedforwardVoltages[0]);
    m_rearRight.setDesiredStateWithFeedForward(desiredStates[0], feedforwardVoltages[0]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    Elastic.Notification notification = new Elastic.Notification(NotificationLevel.INFO, "Gyro Reset", "Gyro heading has been zeroed", 4000);
    Elastic.sendNotification(notification);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0));
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getAngleCorrected() {
    return m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getAngularVelocity() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState()
    };
  }

  private SwerveModuleState[] getDesiredStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getDesiredState(),
      m_frontRight.getDesiredState(),
      m_rearLeft.getDesiredState(),
      m_rearRight.getDesiredState()
    };
  }

  public void setVisionMeasurementStdDevs(boolean isTeleop) {
    if (isTeleop) {
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
    } else {
      m_poseEstimator.setVisionMeasurementStdDevs(
          VecBuilder.fill(9999999, 9999999, 9999999)); // Auto should trust the odometry only
    }
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    LimelightHelpers.SetRobotOrientation(
        VisionConstants.ll_Front,
        m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);
    LimelightHelpers.PoseEstimate mt2Front =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.ll_Front);

    if (mt2FrontConnected) {
      try {
        if (Math.abs(m_gyro.getRate()) > 720) {
          doRejectUpdate = true;
        }
        if (mt2Front.tagCount == 0) {
          doRejectUpdate = true;
        }
        if (!doRejectUpdate) {
          m_poseEstimator.addVisionMeasurement(mt2Front.pose, mt2Front.timestampSeconds);
        }
      } catch (Exception e) {
        System.out.println("WARNING DRIVE: Limelight not connected " + e);
        mt2FrontConnected = false;
      }
    }

    m_poseEstimator.update(
        Rotation2d.fromDegrees(getAngleCorrected()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });

    m_field.setRobotPose(getPose());
  }
}
