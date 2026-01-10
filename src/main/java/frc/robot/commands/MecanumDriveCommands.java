// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.MecanumDrive.MecanumDrive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class MecanumDriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double MAX_LINEAR_SPEED = 3.0; // m/s
  private static final double MAX_ANGULAR_SPEED = 3.0; // rad/s

  // PID constants for angle control
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0; // rad/s
  private static final double ANGLE_MAX_ACCELERATION = 20.0; // rad/s^2

  private MecanumDriveCommands() {}

  /**
   * Applies deadband and squares the input for more precise control at low speeds.
   *
   * @param x X-axis joystick input
   * @param y Y-axis joystick input
   * @return Processed linear velocity as a Translation2d
   */
  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control at low speeds
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field-relative drive command using two joysticks (controlling linear and angular velocities).
   *
   * @param drive The MecanumDrive subsystem
   * @param xSupplier Supplier for X-axis (forward/back) joystick input
   * @param ySupplier Supplier for Y-axis (strafe) joystick input
   * @param omegaSupplier Supplier for rotation joystick input
   * @return The drive command
   */
  public static Command joystickDriveFieldRelative(
      MecanumDrive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity from joysticks
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband and square for precision
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
          omega = Math.copySign(omega * omega, omega);

          // Scale to max speeds
          double vx = linearVelocity.getX() * MAX_LINEAR_SPEED;
          double vy = linearVelocity.getY() * MAX_LINEAR_SPEED;
          double omegaScaled = omega * MAX_ANGULAR_SPEED;

          // Flip for red alliance (field-relative)
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          if (isFlipped) {
            vx = -vx;
            vy = -vy;
          }

          drive.runFieldRelative(vx, vy, omegaScaled);
        },
        drive);
  }

  /**
   * Robot-relative drive command using two joysticks (controlling linear and angular velocities).
   *
   * @param drive The MecanumDrive subsystem
   * @param xSupplier Supplier for X-axis (forward/back) joystick input
   * @param ySupplier Supplier for Y-axis (strafe) joystick input
   * @param omegaSupplier Supplier for rotation joystick input
   * @return The drive command
   */
  public static Command joystickDriveRobotRelative(
      MecanumDrive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity from joysticks
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband and square for precision
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
          omega = Math.copySign(omega * omega, omega);

          // Scale to max speeds
          double vx = linearVelocity.getX() * MAX_LINEAR_SPEED;
          double vy = linearVelocity.getY() * MAX_LINEAR_SPEED;
          double omegaScaled = omega * MAX_ANGULAR_SPEED;

          drive.runRobotRelative(vx, vy, omegaScaled);
        },
        drive);
  }

  /**
   * Field-relative drive command using joystick for linear control and PID for angular control.
   * Useful for snapping to angles, aiming at targets, or absolute rotation control.
   *
   * @param drive The MecanumDrive subsystem
   * @param xSupplier Supplier for X-axis (forward/back) joystick input
   * @param ySupplier Supplier for Y-axis (strafe) joystick input
   * @param rotationSupplier Supplier for the target rotation
   * @return The drive command
   */
  public static Command joystickDriveAtAngle(
      MecanumDrive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller for angle
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              // Get linear velocity from joysticks
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed from PID
              double omega =
                  angleController.calculate(
                      drive.getGyroYaw().getRadians(), rotationSupplier.get().getRadians());

              // Scale linear velocities
              double vx = linearVelocity.getX() * MAX_LINEAR_SPEED;
              double vy = linearVelocity.getY() * MAX_LINEAR_SPEED;

              // Flip for red alliance (field-relative)
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;

              if (isFlipped) {
                vx = -vx;
                vy = -vy;
              }

              drive.runFieldRelative(vx, vy, omega);
            },
            drive)
        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getGyroYaw().getRadians()));
  }

  /**
   * Drive to a specific angle (snap to angle) while allowing translation control.
   *
   * @param drive The MecanumDrive subsystem
   * @param xSupplier Supplier for X-axis (forward/back) joystick input
   * @param ySupplier Supplier for Y-axis (strafe) joystick input
   * @param targetAngle The target angle to snap to
   * @return The drive command
   */
  public static Command snapToAngle(
      MecanumDrive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Rotation2d targetAngle) {
    return joystickDriveAtAngle(drive, xSupplier, ySupplier, () -> targetAngle);
  }

  /**
   * Simple stop command.
   *
   * @param drive The MecanumDrive subsystem
   * @return The stop command
   */
  public static Command stop(MecanumDrive drive) {
    return Commands.runOnce(drive::stop, drive);
  }

  /**
   * Reset the robot's pose to a given pose.
   *
   * @param drive The MecanumDrive subsystem
   * @param pose The pose to reset to
   * @return The reset command
   */
  public static Command resetPose(MecanumDrive drive, Pose2d pose) {
    return Commands.runOnce(() -> drive.resetPose(pose), drive).ignoringDisable(true);
  }

  /**
   * Reset the robot's heading to zero (keeps position).
   *
   * @param drive The MecanumDrive subsystem
   * @return The reset heading command
   */
  public static Command resetHeading(MecanumDrive drive) {
    return Commands.runOnce(
            () -> drive.resetPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
            drive)
        .ignoringDisable(true);
  }
}