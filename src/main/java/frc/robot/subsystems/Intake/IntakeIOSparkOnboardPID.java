package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSparkOnboardPID implements IntakeIO {
    private final SparkMax leftDoorMotor;  // NEO 550 motor - LEADER
    private final SparkMax rightDoorMotor; // NEO 550 motor - FOLLOWER
    private final VictorSPX intakeWheel;
    
    private final SparkAbsoluteEncoder doorEncoder; // Reference to absolute encoder

    // Safety constants
    private static final double MIN_DOOR_POSITION = 0.0; // radians
    private static final double MAX_DOOR_POSITION = Math.toRadians(130); // radians
    private static final double MAX_CURRENT_AMPS = 20.0; // current limit for stall detection
    private static final double STALL_VELOCITY_THRESHOLD = 0.05; // rad/s

    private boolean safetyTripped = false;

    public IntakeIOSparkOnboardPID() { 
        // Initialize motors
        leftDoorMotor = new SparkMax(IntakeConstants.INTAKE_LEFT_DOOR_MOTOR_ID, MotorType.kBrushless);
        rightDoorMotor = new SparkMax(IntakeConstants.INTAKE_RIGHT_DOOR_MOTOR_ID, MotorType.kBrushless);
        intakeWheel = new VictorSPX(IntakeConstants.INTAKE_WHEEL_MOTOR_ID); 

        // LEFT door motor is the leader with an absolute encoder for position feedback
        var doorConfig = new SparkMaxConfig();
        doorConfig
            .closedLoop
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, Math.PI*2)
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(IntakeConstants.INTAKE_DOOR_kP, IntakeConstants.INTAKE_DOOR_kI, IntakeConstants.INTAKE_DOOR_kD);
        // doorConfig.closedLoop.apply(new FeedForwardConfig().kCos(0.8).kCosRatio(1));
        doorConfig.absoluteEncoder.positionConversionFactor(2 * Math.PI);
        doorConfig.inverted(IntakeConstants.IS_INTAKE_DIRECTION_INVERTED);
        doorConfig.absoluteEncoder.inverted(IntakeConstants.IS_INTAKE_ENCODER_INVERTED);
        
        // Add current limit for safety
        doorConfig.smartCurrentLimit(50);
        
        leftDoorMotor.configure(doorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // RIGHT door motor follows the LEFT door motor, inverted
        var followerConfig = new SparkMaxConfig();
        followerConfig.follow(leftDoorMotor, true);
        followerConfig.smartCurrentLimit(50);
        rightDoorMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Get reference to the absolute encoder on the leader
        doorEncoder = leftDoorMotor.getAbsoluteEncoder();
    }   

    @Override
    public void setIntakeWheelSpeedPercentOut(double percentOutput) {
        intakeWheel.set(VictorSPXControlMode.PercentOutput, percentOutput);
    }

    @Override
    public void setIntakeDoorPosition(double position, IntakeIOInputs inputs) {
        // Safety: Don't move if safety has been tripped
        if (safetyTripped) {
            System.out.println("SAFETY: Intake door disabled due to safety trip");
            leftDoorMotor.stopMotor();
            return;
        }

        // Safety: Clamp position to safe range
        double safePosition = MathUtil.clamp(position, MIN_DOOR_POSITION, MAX_DOOR_POSITION);
        
        if (position != safePosition) {
            System.out.println("SAFETY: Clamped intake position from " + position + " to " + safePosition);
        }

        // Control the LEADER motor (leftDoorMotor) using standard position PID
        leftDoorMotor.getClosedLoopController().setReference(safePosition, ControlType.kPosition); 
    }

    public void stop(IntakeIOInputs inputs) { 
        leftDoorMotor.stopMotor();
        intakeWheel.set(VictorSPXControlMode.PercentOutput, 0);
    }

    /** Call this to reset safety trip and re-enable the intake */
    public void resetSafety() {
        safetyTripped = false;
        System.out.println("Intake safety reset");
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) { 
        // Read from ABSOLUTE encoder (not relative encoder)
        inputs.currentDoorPosition = doorEncoder.getPosition();
        
        // Read velocities from relative encoders (these are fine)
        inputs.rightIntakeWheelSpeedRPM = rightDoorMotor.getEncoder().getVelocity();
        inputs.leftIntakeWheelSpeedRPM = leftDoorMotor.getEncoder().getVelocity();

        // Safety check: detect stall (high current + low velocity)
        double current = leftDoorMotor.getOutputCurrent();
        double velocity = Math.abs(leftDoorMotor.getEncoder().getVelocity());
        
        if (current > MAX_CURRENT_AMPS && velocity < STALL_VELOCITY_THRESHOLD) {
            safetyTripped = true;
            leftDoorMotor.stopMotor();
            System.out.println("SAFETY: Intake stall detected! Current: " + current + "A, Velocity: " + velocity);
        }
    }
}