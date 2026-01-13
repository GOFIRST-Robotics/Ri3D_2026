package frc.robot.subsystems.drive.MecanumDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.Gyro.GyroIO;
import frc.robot.subsystems.Gyro.GyroIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

/**
 * Mecanum drive subsystem built from four {@link MecanumModule}s.
 */
public class MecanumDrive extends SubsystemBase {
  // Module order: 0=FL, 1=FR, 2=BL, 3=BR
  private final MecanumModule[] modules = new MecanumModule[4];

  private final MecanumDriveKinematics kinematics;
  private final MecanumDriveOdometry odometry;
  private final MecanumDrivePoseEstimator odometryEstimator;

  // Gyro IO
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  // Alert for disconnected gyro
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, odometry may be inaccurate.", AlertType.kWarning);

  /**
   * @param moduleIOs Array of 4 IO implementations in module order: FL, FR, BL, BR.
   * @param gyroIO Gyro IO implementation for heading.
   */
  public MecanumDrive(MecanumModuleIO[] moduleIOs, GyroIO gyroIO) {
    if (moduleIOs.length != 4) {
      throw new IllegalArgumentException("Expected 4 mecanum modules (FL, FR, BL, BR).");
    }

    this.gyroIO = gyroIO;

    for (int i = 0; i < 4; i++) {
      modules[i] = new MecanumModule(moduleIOs[i], i);
    }

    // Wheel locations (robot-relative)
    Translation2d fl = new Translation2d(Constants.WHEEL_BASE / 2.0, Constants.TRACK_WIDTH / 2.0);
    Translation2d fr = new Translation2d(Constants.WHEEL_BASE / 2.0, -Constants.TRACK_WIDTH / 2.0);
    Translation2d bl = new Translation2d(-Constants.WHEEL_BASE / 2.0, Constants.TRACK_WIDTH / 2.0);
    Translation2d br = new Translation2d(-Constants.WHEEL_BASE / 2.0, -Constants.TRACK_WIDTH / 2.0);

    kinematics = new MecanumDriveKinematics(fl, fr, bl, br);

    // Initialize gyro inputs before creating odometry
    gyroIO.updateInputs(gyroInputs);

    odometry =
        new MecanumDriveOdometry(
            kinematics,
            gyroInputs.yawPosition,
            getWheelPositions(),
            new Pose2d());

    odometryEstimator = new MecanumDrivePoseEstimator(
        kinematics,
        gyroInputs.yawPosition,
        getWheelPositions(),
        new Pose2d(),
        Constants.DRIVE_STANDARD_DEVIATIONS,
        Constants.DRIVE_VISION_MEASUREMENT_STD_DEVS);
  }

  @Override
  public void periodic() {
    // Update gyro inputs
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    // Update modules + log per-module inputs
    for (var module : modules) {
      module.periodic();
    }

    // Update odometry
    odometry.update(gyroInputs.yawPosition, getWheelPositions());

    // Update odometry estimator
    odometryEstimator.update(getGyroYaw(), getWheelPositions());

    // Update gyro disconnected alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    // Log outputs
    Logger.recordOutput("Drive/Mecanum/Pose", odometry.getPoseMeters());
    Logger.recordOutput("Drive/Mecanum/EstimatedPose", odometryEstimator.getEstimatedPosition());
    Logger.recordOutput("Drive/Mecanum/Heading", gyroInputs.yawPosition.getRadians());
  }

  /** Returns the current gyro heading. */
  public Rotation2d getGyroYaw() {
    return gyroInputs.yawPosition;
  }

  /** Robot pose from odometry. */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public Pose2d getEstimatedPose() {
    return odometryEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
        odometry.resetPosition(gyroInputs.yawPosition, getWheelPositions(), pose);
    odometryEstimator.resetPosition(gyroInputs.yawPosition, getWheelPositions(), pose);
  }

  /** Wheel distances (meters) used by WPILib odometry. */
  public MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(
        modules[0].getWheelPositionMeters(),
        modules[1].getWheelPositionMeters(),
        modules[2].getWheelPositionMeters(),
        modules[3].getWheelPositionMeters());
  }

  /** Wheel speeds (m/s) convenience accessor. */
  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        modules[0].getWheelVelocityMetersPerSec(),
        modules[1].getWheelVelocityMetersPerSec(),
        modules[2].getWheelVelocityMetersPerSec(),
        modules[3].getWheelVelocityMetersPerSec());
  }

  /** Drives robot-relative. */
  public void runRobotRelative(double vxMetersPerSec, double vyMetersPerSec, double omegaRadPerSec) {
    runRobotRelative(new ChassisSpeeds(vxMetersPerSec, vyMetersPerSec, omegaRadPerSec));
  }

  /** Drives robot-relative. */
  public void runRobotRelative(ChassisSpeeds speeds) {
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    setWheelSpeeds(wheelSpeeds);
  }

  /** Drives field-relative (uses gyro). */
  public void runFieldRelative(double vxMetersPerSec, double vyMetersPerSec, double omegaRadPerSec) {
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            vxMetersPerSec, vyMetersPerSec, omegaRadPerSec, gyroInputs.yawPosition);
    runRobotRelative(speeds);
  }

  /** Sets wheel speeds using each module's closed-loop velocity (m/s). */
  public void setWheelSpeeds(MecanumDriveWheelSpeeds speeds) {
    modules[0].runVelocityMetersPerSec(speeds.frontLeftMetersPerSecond);
    modules[1].runVelocityMetersPerSec(speeds.frontRightMetersPerSecond);
    modules[2].runVelocityMetersPerSec(speeds.rearLeftMetersPerSecond);
    modules[3].runVelocityMetersPerSec(speeds.rearRightMetersPerSecond);

    Logger.recordOutput("Drive/Mecanum/Setpoints/frontLeftMps", speeds.frontLeftMetersPerSecond);
    Logger.recordOutput("Drive/Mecanum/Setpoints/frontRightMps", speeds.frontRightMetersPerSecond);
    Logger.recordOutput("Drive/Mecanum/Setpoints/rearLeftMps", speeds.rearLeftMetersPerSecond);
    Logger.recordOutput("Drive/Mecanum/Setpoints/rearRightMps", speeds.rearRightMetersPerSecond);
  }

  /** Open-loop voltage (useful for characterization/debug). */
  public void runOpenLoopVolts(double fl, double fr, double bl, double br) {
    modules[0].runOpenLoopVolts(fl);
    modules[1].runOpenLoopVolts(fr);
    modules[2].runOpenLoopVolts(bl);
    modules[3].runOpenLoopVolts(br);
  }

  public void stop() {
    for (var module : modules) {
      module.stop();
    }
  }




  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    odometryEstimator.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
  }

}