package frc.robot.subsystems.drive.MecanumDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.SparkOdometryThread;
import org.littletonrobotics.junction.Logger;

/**
 * Mecanum drive subsystem built from four {@link MecanumModule}s.
 *
 * <p>Matches the swerve Drive style:
 * - owns modules
 * - runs module periodic + logs
 * - exposes chassis control (robot-relative and field-relative)
 * - maintains WPILib mecanum odometry using module distances
 */
public class MecanumDrive extends SubsystemBase {
  // Module order: 0=FL, 1=FR, 2=BL, 3=BR
  private final MecanumModule[] modules = new MecanumModule[4];

  private final MecanumDriveKinematics kinematics;
  private final MecanumDriveOdometry odometry;

  private final Alert odometryThreadNotStartedAlert =
      new Alert("SparkOdometryThread is not started (odometry sampling disabled).", AlertType.kWarning);

  /**
   * @param moduleIOs Array of 4 IO implementations in module order: FL, FR, BL, BR.
   *     (This mirrors the swerve Drive constructor style of passing IO by module.)
   * @param gyroSupplier Supplies current robot heading (CCW+), used for field-relative control and odometry.
   */
  public MecanumDrive(MecanumModuleIO[] moduleIOs, java.util.function.Supplier<Rotation2d> gyroSupplier) {
    if (moduleIOs.length != 4) {
      throw new IllegalArgumentException("Expected 4 mecanum modules (FL, FR, BL, BR).");
    }

    for (int i = 0; i < 4; i++) {
      modules[i] = new MecanumModule(moduleIOs[i], i);
    }

    // Wheel locations (robot-relative)
    Translation2d fl = new Translation2d(Constants.WHEEL_BASE / 2.0, Constants.TRACK_WIDTH / 2.0);
    Translation2d fr = new Translation2d(Constants.WHEEL_BASE / 2.0, -Constants.TRACK_WIDTH / 2.0);
    Translation2d bl = new Translation2d(-Constants.WHEEL_BASE / 2.0, Constants.TRACK_WIDTH / 2.0);
    Translation2d br = new Translation2d(-Constants.WHEEL_BASE / 2.0, -Constants.TRACK_WIDTH / 2.0);

    kinematics = new MecanumDriveKinematics(fl, fr, bl, br);

    odometry =
        new MecanumDriveOdometry(
            kinematics,
            gyroSupplier.get(),
            getWheelPositions(),
            new Pose2d());

    // Start odometry sampling thread for Sparks (real+sim; safe on replay too, but typically unnecessary)
    if (Constants.currentMode != Mode.REPLAY) {
      SparkOdometryThread.getInstance().start();
      odometryThreadNotStartedAlert.set(false);
    } else {
      odometryThreadNotStartedAlert.set(true);
    }

    // Store gyro supplier
    this.gyroSupplier = gyroSupplier;
  }

  private final java.util.function.Supplier<Rotation2d> gyroSupplier;

  @Override
  public void periodic() {
    // Update modules + log per-module inputs
    for (var module : modules) {
      module.periodic();
    }

    // Use the most recent sample set for odometry:
    // We use per-cycle wheel positions (meters). SparkOdometryThread gives timestamps too,
    // but WPILib's MecanumDriveOdometry doesn't support multi-sample time sync like swerve in this template.
    odometry.update(gyroSupplier.get(), getWheelPositions());

    Logger.recordOutput("Drive/Mecanum/Pose", odometry.getPoseMeters());
    Logger.recordOutput("Drive/Mecanum/Heading", gyroSupplier.get().getRadians());
  }

  /** Robot pose from odometry. */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(gyroSupplier.get(), getWheelPositions(), pose);
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
        ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSec, vyMetersPerSec, omegaRadPerSec, gyroSupplier.get());
    runRobotRelative(speeds);
  }

  /** Sets wheel speeds using each module's closed-loop velocity (m/s). */
  public void setWheelSpeeds(MecanumDriveWheelSpeeds speeds) {
    // NOTE: If your right side or diagonals are inverted (like in your old code),
    // fix it in motor inversion OR here. Prefer motor inversion in IO.
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
}