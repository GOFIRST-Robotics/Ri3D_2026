package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSpark implements IntakeIO {
    private final SparkMax leftDoorMotor;  // NEO 550 motor - LEADER
    private final SparkMax rightDoorMotor; // NEO 550 motor - FOLLOWER
    private final VictorSPX intakeWheel;
    
    private final SparkAbsoluteEncoder doorEncoder; // Reference to absolute encoder

    public IntakeIOSpark() { 
        // Initialize motors
        leftDoorMotor = new SparkMax(IntakeConstants.INTAKE_LEFT_DOOR_MOTOR_ID, MotorType.kBrushless);
        rightDoorMotor = new SparkMax(IntakeConstants.INTAKE_RIGHT_DOOR_MOTOR_ID, MotorType.kBrushless);
        intakeWheel = new VictorSPX(IntakeConstants.INTAKE_WHEEL_MOTOR_ID); 

        // LEFT door motor is the leader with an absolute encoder for position feedback
        var doorConfig = new SparkMaxConfig();
        doorConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(IntakeConstants.INTAKE_DOOR_kP, 0.0, 0.0);
        doorConfig.closedLoop.apply(new FeedForwardConfig().kCos(0.8));
        doorConfig.absoluteEncoder.positionConversionFactor(2 * Math.PI);
        doorConfig.inverted(IntakeConstants.IS_INTAKE_DIRECTION_INVERTED);
        doorConfig.absoluteEncoder.inverted(IntakeConstants.IS_INTAKE_ENCODER_INVERTED);
        doorConfig.softLimit
            .forwardSoftLimit(Units.degreesToRadians(125))
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(0)
            .reverseSoftLimitEnabled(true);
        doorConfig.closedLoop.maxMotion
            .cruiseVelocity(0.5)
            .maxAcceleration(0.4)
            .allowedProfileError(0.5);
        leftDoorMotor.configure(doorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // RIGHT door motor follows the LEFT door motor, inverted
        var followerConfig = new SparkMaxConfig();
        followerConfig.follow(leftDoorMotor, true);
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
        // Control the LEADER motor (leftDoorMotor)
        leftDoorMotor.getClosedLoopController().setSetpoint(position, ControlType.kMAXMotionPositionControl); 
    }

    public void stop(IntakeIOInputs inputs) { 
        leftDoorMotor.stopMotor();
        intakeWheel.set(VictorSPXControlMode.PercentOutput, 0);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) { 
        // Read from ABSOLUTE encoder (not relative encoder)
        inputs.currentDoorPosition = doorEncoder.getPosition();
        
        // Read velocities from relative encoders (these are fine)
        inputs.rightIntakeWheelSpeedRPM = rightDoorMotor.getEncoder().getVelocity();
        inputs.leftIntakeWheelSpeedRPM = leftDoorMotor.getEncoder().getVelocity();
    }
}