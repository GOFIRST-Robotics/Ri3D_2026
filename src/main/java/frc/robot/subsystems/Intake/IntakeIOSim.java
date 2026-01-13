package frc.robot.subsystems.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * WPILib simulation implementation of {@link IntakeIO}.
 *
 * <p>This sim models:
 * <ul>
 *   <li>Two intake wheels as flywheels (velocity in RPM)</li>
 *   <li>An intake "door" as a single-jointed arm (position in degrees)</li>
 * </ul>
 *
 * <p>All constants here are placeholders — tune them to match your mechanism.
 */
public class IntakeIOSim implements IntakeIO {

  // -------------------- Intake Wheels --------------------
  // Simple flywheel model for each side.
  // Tune gearing and MOI to roughly match your intake.
  private static final double kWheelGearing = 500.0;
  private static final double kWheelMoiKgM2 = 0.0005;

  private final FlywheelSim rightWheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), kWheelMoiKgM2, kWheelGearing),
          DCMotor.getNeo550(1));
  private final FlywheelSim leftWheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), kWheelMoiKgM2, kWheelGearing),
          DCMotor.getNeo550(1));

  // Percent output command [-1, 1]
  private double wheelPercentOut = 0.0;

  // -------------------- Door --------------------
  // Model the door as a single-jointed arm.
  // These are placeholder values (arm length, mass, min/max angles).
  private static final double kDoorGearing = 500.0;
  private static final double kDoorLengthMeters = 0.25;
  private static final double kDoorMassKg = 1.0;
  private static final double kDoorMinAngleRad = Math.toRadians(-10.0);
  private static final double kDoorMaxAngleRad = Math.toRadians(90.0);

  private final SingleJointedArmSim doorSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          kDoorGearing,
          SingleJointedArmSim.estimateMOI(kDoorLengthMeters, kDoorMassKg),
          kDoorLengthMeters,
          kDoorMinAngleRad,
          kDoorMaxAngleRad,
          true,
          0.0);

  // Door closed-loop position control (in radians)
  private final PIDController doorPid = new PIDController(12.0, 0.0, 0.8);
  private double desiredDoorPosRad = 0.0;

  // -------------------- Timing --------------------
  private double lastTimestampSec = Timer.getFPGATimestamp();

  public IntakeIOSim() {
    // Keep PID stable if sim timing jitter occurs
    doorPid.setTolerance(Math.toRadians(1.0));
  }

  /** Run the Intake Belt/Wheels at a specified percent output (open-loop). */
  @Override
  public void setIntakeWheelSpeedPercentOut(double percentOutput) {
    wheelPercentOut = MathUtil.clamp(percentOutput, -1.0, 1.0);
  }

  /**
   * Set the desired door position in degrees.
   *
   * <p>In sim, we drive the arm to this target using a PID loop in {@link #updateInputs}.
   */
  @Override
  public void setIntakeDoorPosition(double position, IntakeIOInputs inputs) {
    // Treat "position" as degrees (matching IntakeIOInputs fields)
    desiredDoorPosRad = Math.toRadians(position);
    inputs.desiredDoorPosition = position;
  }

  /** Update the simulation and populate the inputs struct. */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    final double now = Timer.getFPGATimestamp();
    double dt = now - lastTimestampSec;
    lastTimestampSec = now;

    // If something weird happens (e.g., sim paused), fall back to nominal loop time.
    if (dt <= 0.0 || dt > 0.1) {
      dt = 0.02;
    }

    // ----- Wheels -----
    final double wheelVoltage = wheelPercentOut * 12.0;
    rightWheelSim.setInputVoltage(wheelVoltage);
    leftWheelSim.setInputVoltage(wheelVoltage);

    rightWheelSim.update(dt);
    leftWheelSim.update(dt);

    inputs.rightIntakeWheelSpeedRPM = rightWheelSim.getAngularVelocityRPM();
    inputs.leftIntakeWheelSpeedRPM = leftWheelSim.getAngularVelocityRPM();

    // ----- Door -----
    // Drive the arm toward the desired position with PID.
    final double currentRad = doorSim.getAngleRads();
    double pidVolts = doorPid.calculate(currentRad, desiredDoorPosRad);
    pidVolts = MathUtil.clamp(pidVolts, -12.0, 12.0);

    doorSim.setInputVoltage(pidVolts);
    doorSim.update(dt);

    inputs.currentDoorPosition = Math.toDegrees(doorSim.getAngleRads());
    inputs.desiredDoorPosition = Math.toDegrees(desiredDoorPosRad);
  }

  @Override
  public void stop() {
    wheelPercentOut = 0.0;
  }
}
