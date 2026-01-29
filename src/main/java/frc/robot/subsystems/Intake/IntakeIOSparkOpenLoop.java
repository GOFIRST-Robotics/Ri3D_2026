package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.IntakeConstants;

public class IntakeIOSparkOpenLoop implements IntakeIO {
    private final SparkMax leftDoorMotor;  // NEO 550 motor - LEADER
    private final SparkMax rightDoorMotor; // NEO 550 motor - FOLLOWER
    private final VictorSPX intakeWheel;
    
    private final SparkAbsoluteEncoder doorEncoder; // Reference to absolute encoder

    // Feedforward constants
    private static final double ENCODER_AT_HORIZONTAL = 0.33; // radians - when mechanism is at 90° (horizontal)
    private static final double ENCODER_AT_VERTICAL = 2.05;   // radians - when mechanism is straight up
    private static final double KG = 0.15; // Gravity compensation gain - tune this value

    private double manualSpeed = 0.0; // Store manual speed command

    public IntakeIOSparkOpenLoop() { 
        // Initialize motors
        leftDoorMotor = new SparkMax(IntakeConstants.INTAKE_LEFT_DOOR_MOTOR_ID, MotorType.kBrushless);
        rightDoorMotor = new SparkMax(IntakeConstants.INTAKE_RIGHT_DOOR_MOTOR_ID, MotorType.kBrushless);
        intakeWheel = new VictorSPX(IntakeConstants.INTAKE_WHEEL_MOTOR_ID); 

        // LEFT door motor config - no closed loop, just basic setup
        var doorConfig = new SparkMaxConfig();
        doorConfig.absoluteEncoder.positionConversionFactor(2 * Math.PI);
        doorConfig.inverted(IntakeConstants.IS_INTAKE_DIRECTION_INVERTED);
        doorConfig.absoluteEncoder.inverted(IntakeConstants.IS_INTAKE_ENCODER_INVERTED);
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

    /**
     * Calculates gravity compensation feedforward based on encoder position.
     * Uses cosine of the mechanism angle relative to vertical.
     * @param encoderPosition current encoder reading in radians
     * @return feedforward output to counteract gravity
     */
    private double calculateGravityFeedforward(double encoderPosition) {
        // Convert encoder position to mechanism angle relative to vertical
        // At ENCODER_AT_VERTICAL (2.05), mechanism is vertical (0° from vertical)
        // At ENCODER_AT_HORIZONTAL (0.33), mechanism is horizontal (90° from vertical)
        double mechanismAngleFromVertical = (ENCODER_AT_VERTICAL - encoderPosition) 
            / (ENCODER_AT_VERTICAL - ENCODER_AT_HORIZONTAL) * (Math.PI / 2);
        
        // Gravity torque is proportional to sin(angle from vertical)
        // which equals cos(angle from horizontal)
        return KG * Math.sin(mechanismAngleFromVertical);
    }

    @Override
    public void setIntakeWheelSpeedPercentOut(double percentOutput) {
        intakeWheel.set(VictorSPXControlMode.PercentOutput, percentOutput);
    }

    @Override
    public void setIntakeDoorPosition(double position, IntakeIOInputs inputs) {
        // Open loop: ignore position parameter
        // Use setDoorSpeed() instead for open loop control
    }

    /**
     * Set door motor speed directly (open loop control)
     * Gravity feedforward is automatically applied
     * @param percentOutput -1.0 to 1.0 (0 = hold position with gravity comp only)
     */
    public void setDoorSpeed(double percentOutput) {
        manualSpeed = percentOutput;
    }

    public void stop(IntakeIOInputs inputs) { 
        manualSpeed = 0.0;
        leftDoorMotor.stopMotor();
        intakeWheel.set(VictorSPXControlMode.PercentOutput, 0);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) { 
        // Read from ABSOLUTE encoder (for monitoring and feedforward)
        inputs.currentDoorPosition = doorEncoder.getPosition();
        
        // Read velocities from relative encoders
        inputs.rightIntakeWheelSpeedRPM = rightDoorMotor.getEncoder().getVelocity();
        inputs.leftIntakeWheelSpeedRPM = leftDoorMotor.getEncoder().getVelocity();

        // Apply gravity feedforward + manual speed command
        double feedforward = calculateGravityFeedforward(inputs.currentDoorPosition);
        double totalOutput = manualSpeed + feedforward;
        leftDoorMotor.set(totalOutput);
    }
}