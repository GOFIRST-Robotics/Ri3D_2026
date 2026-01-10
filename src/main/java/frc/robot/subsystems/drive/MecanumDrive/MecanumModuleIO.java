package frc.robot.subsystems.drive.MecanumDrive;

import org.littletonrobotics.junction.AutoLog;

public interface MecanumModuleIO {
  @AutoLog
  public static class MecanumModuleIOInputs {
    public boolean connected = false;

    /** Wheel position in radians at the motor/encoder measurement point (after conversion factor). */
    public double wheelPositionRad = 0.0;

    /** Wheel velocity in rad/sec at the motor/encoder measurement point (after conversion factor). */
    public double wheelVelocityRadPerSec = 0.0;

    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;

    /** High-rate samples for odometry. */
    public double[] odometryTimestamps = new double[] {};
    public double[] odometryWheelPositionsRad = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(MecanumModuleIOInputs inputs) {}

  /** Run the wheel motor at the specified voltage (open-loop). */
  public default void setOpenLoopVolts(double volts) {}

  /** Run the wheel motor at the specified wheel angular velocity (closed-loop). */
  public default void setWheelVelocity(double velocityRadPerSec) {}

  /** Stop outputs. */
  public default void stop() {
    setOpenLoopVolts(0.0);
  }
}