package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeIOSparkOnboardPIDWithTuning implements IntakeIO {
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

    // Logged tunable numbers for PID
    private final LoggedNetworkNumber tunableKP;
    private final LoggedNetworkNumber tunableKI;
    private final LoggedNetworkNumber tunableKD;
    private final LoggedNetworkNumber tunableKCos;

    // Track last values to detect changes
    private double lastKP;
    private double lastKI;
    private double lastKD;
    private double lastKCos;

    public IntakeIOSparkOnboardPIDWithTuning() { 
        // Initialize motors
        leftDoorMotor = new SparkMax(IntakeConstants.INTAKE_LEFT_DOOR_MOTOR_ID, MotorType.kBrushless);
        rightDoorMotor = new SparkMax(IntakeConstants.INTAKE_RIGHT_DOOR_MOTOR_ID, MotorType.kBrushless);
        intakeWheel = new VictorSPX(IntakeConstants.INTAKE_WHEEL_MOTOR_ID); 

        // Initialize tunable numbers with default values
        tunableKP = new LoggedNetworkNumber("/Tuning/Intake/kP", IntakeConstants.INTAKE_DOOR_kP);
        tunableKI = new LoggedNetworkNumber("/Tuning/Intake/kI", IntakeConstants.INTAKE_DOOR_kI);
        tunableKD = new LoggedNetworkNumber("/Tuning/Intake/kD", IntakeConstants.INTAKE_DOOR_kD);
        tunableKCos = new LoggedNetworkNumber("/Tuning/Intake/kCos", 0.8);

        // Store initial values
        lastKP = IntakeConstants.INTAKE_DOOR_kP;
        lastKI = IntakeConstants.INTAKE_DOOR_kI;
        lastKD = IntakeConstants.INTAKE_DOOR_kD;
        lastKCos = 0.8;

        // LEFT door motor is the leader with an absolute encoder for position feedback
        var doorConfig = new SparkMaxConfig();
        doorConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(lastKP, lastKI, lastKD);
        doorConfig.closedLoop.apply(new FeedForwardConfig().kCos(lastKCos).kCosRatio(1));
        doorConfig.absoluteEncoder.positionConversionFactor(2 * Math.PI);
        doorConfig.inverted(IntakeConstants.IS_INTAKE_DIRECTION_INVERTED);
        doorConfig.absoluteEncoder.inverted(IntakeConstants.IS_INTAKE_ENCODER_INVERTED);
        
        // Add current limit for safety
        doorConfig.smartCurrentLimit(25);
        
        leftDoorMotor.configure(doorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // RIGHT door motor follows the LEFT door motor, inverted
        var followerConfig = new SparkMaxConfig();
        followerConfig.follow(leftDoorMotor, true);
        followerConfig.smartCurrentLimit(25);
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

        // Check for tunable value changes and update config if needed
        updateTunableValues();
    }

    /** Check if tunable values have changed and update motor config accordingly */
    private void updateTunableValues() {
        double currentKP = tunableKP.get();
        double currentKI = tunableKI.get();
        double currentKD = tunableKD.get();
        double currentKCos = tunableKCos.get();

        boolean pidChanged = currentKP != lastKP || currentKI != lastKI || currentKD != lastKD;
        boolean ffChanged = currentKCos != lastKCos;

        if (pidChanged || ffChanged) {
            var config = new SparkMaxConfig();
            
            if (pidChanged) {
                config.closedLoop.pid(currentKP, currentKI, currentKD, ClosedLoopSlot.kSlot0);
                lastKP = currentKP;
                lastKI = currentKI;
                lastKD = currentKD;
                System.out.println("Intake PID updated - kP: " + currentKP + ", kI: " + currentKI + ", kD: " + currentKD);
            }

            if (ffChanged) {
                config.closedLoop.apply(new FeedForwardConfig().kCos(currentKCos).kCosRatio(1));
                lastKCos = currentKCos;
                System.out.println("Intake FF updated - kCos: " + currentKCos);
            }

            // Apply only the changed values without resetting other parameters
            leftDoorMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }
}