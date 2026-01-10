package frc.robot.subsystems.drive.MecanumDrive;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

/**
 * A single mecanum "module" (one wheel + one drive motor/encoder).
 *
 * <p>Mirrors the structure of {@code frc.robot.subsystems.drive.Module} used for swerve:
 * - IO layer provides sensor readings and motor control
 * - this class logs inputs, tracks connection alerts, and exposes wheel states/odometry samples
 */
public class MecanumModule {
  private final MecanumModuleIO io;
  private final MecanumModuleIOInputsAutoLogged inputs = new MecanumModuleIOInputsAutoLogged();
  private final int index;

  private final Alert disconnectedAlert;

  public MecanumModule(MecanumModuleIO io, int index) {
    this.io = io;
    this.index = index;
    disconnectedAlert =
        new Alert(
            "Disconnected mecanum drive motor on wheel " + Integer.toString(index) + ".",
            AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/MecanumModule" + Integer.toString(index), inputs);

    // Alerts
    disconnectedAlert.set(!inputs.connected);
  }

  /** Open-loop voltage control. */
  public void runOpenLoopVolts(double volts) {
    io.setOpenLoopVolts(volts);
  }

  /** Closed-loop wheel velocity control in meters/sec (converted internally to rad/sec). */
  public void runVelocityMetersPerSec(double wheelMetersPerSec) {
    double wheelRadiusMeters = Constants.WHEEL_DIAMETER / 2.0;
    io.setWheelVelocity(wheelMetersPerSec / wheelRadiusMeters);
  }

  /** Disables outputs. */
  public void stop() {
    io.stop();
  }

  /** Wheel position in meters (integrated distance). */
  public double getWheelPositionMeters() {
    double wheelRadiusMeters = Constants.WHEEL_DIAMETER / 2.0;
    return inputs.wheelPositionRad * wheelRadiusMeters;
  }

  /** Wheel velocity in meters/sec. */
  public double getWheelVelocityMetersPerSec() {
    double wheelRadiusMeters = Constants.WHEEL_DIAMETER / 2.0;
    return inputs.wheelVelocityRadPerSec * wheelRadiusMeters;
  }

  /** Wheel position in radians (for characterization/debug). */
  public double getWheelPositionRad() {
    return inputs.wheelPositionRad;
  }

  /** Wheel velocity in rad/sec (for characterization/debug). */
  public double getWheelVelocityRadPerSec() {
    return inputs.wheelVelocityRadPerSec;
  }

}