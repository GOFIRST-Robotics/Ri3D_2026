package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSpark implements IntakeIO {
    private final SparkMax rightDoorMotor; // NEO 550 motor 
    private final SparkMax leftDoorMotor;  // NEO 550 motor
    private final VictorSPX intakeWheel; // Victor SP motor for inner intake wheel
    
    private LoggedNetworkNumber changeableIntakekP; // kP for the intake door position controller
    private double previousIntakekP = IntakeConstants.INTAKE_DOOR_kP;


    /** Subsystem for controlling Intake */
    public IntakeIOSpark() { 

        // Initialize motors
        rightDoorMotor = new SparkMax(IntakeConstants.INTAKE_RIGHT_DOOR_MOTOR_ID, MotorType.kBrushless); 
        leftDoorMotor = new SparkMax(IntakeConstants.INTAKE_LEFT_DOOR_MOTOR_ID, MotorType.kBrushless);
        intakeWheel = new VictorSPX(IntakeConstants.INTAKE_WHEEL_MOTOR_ID); 

        

        // Configure intake door motors
        changeableIntakekP = new LoggedNetworkNumber("/Tuning/Intake/ChangeableIntakeKP", IntakeConstants.INTAKE_DOOR_kP);
        

        // Right door motor is the leader with an absolute encoder for position feedback
        var doorConfig = new SparkMaxConfig();
        doorConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(changeableIntakekP.getAsDouble(), 0.0, 0.0);
        doorConfig.closedLoop.apply(new FeedForwardConfig().kCos(0.8));
        doorConfig.absoluteEncoder.positionConversionFactor(2 * Math.PI); // Radians to degrees
        doorConfig.inverted(IntakeConstants.IS_INTAKE_DIRECTION_INVERTED);
        doorConfig.absoluteEncoder.inverted(IntakeConstants.IS_INTAKE_ENCODER_INVERTED);
        doorConfig.softLimit
            .forwardSoftLimit(Units.degreesToRadians(125))
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(0)
            .reverseSoftLimitEnabled(true);
        doorConfig.closedLoop.maxMotion
            .cruiseVelocity(0.5) //0.5 radians per second
            .maxAcceleration(0.4) //radian accelerations
            //.cruiseVelocity(10000)
            .allowedProfileError(0.5); //0.5 radians off
        rightDoorMotor.configure(doorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Left door motor follows the right door motor, inverted
        var followerConfig = new SparkMaxConfig();
        followerConfig.follow(rightDoorMotor);
        followerConfig.inverted(true);
        leftDoorMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }   

    @Override
    public void setIntakeWheelSpeedPercentOut(double percentOutput) {
        // Set the intake wheel motor speed as a percent output
        intakeWheel.set(VictorSPXControlMode.PercentOutput, percentOutput);
    }

    public void setIntakeDoorPosition(double position, IntakeIOInputs inputs) {
        // Set the desired position for the intake door (in degrees)
        if(position > IntakeConstants.FINAL_INTAKE_DOOR_POSITION) {
            position = IntakeConstants.FINAL_INTAKE_DOOR_POSITION;
        }
        rightDoorMotor.getClosedLoopController().setSetpoint(position, ControlType.kPosition); 
    }

    public void stop(IntakeIOInputs inputs) { 

    if(!(inputs.currentDoorPosition > IntakeConstants.FINAL_INTAKE_DOOR_POSITION)) {
        // If the door is near the final position, hold it there
        rightDoorMotor.stopMotor();
        intakeWheel.set(VictorSPXControlMode.PercentOutput, 0);
    }
        
      rightDoorMotor.getClosedLoopController().setSetpoint(IntakeConstants.FINAL_INTAKE_DOOR_POSITION, ControlType.kPosition);
    }




    @Override
    public void updateInputs(IntakeIOInputs inputs) { 
        // Update kP if it has changed from previous value
        if(changeableIntakekP.getAsDouble() != previousIntakekP) {
            rightDoorMotor.getClosedLoopController().setSetpoint(changeableIntakekP.getAsDouble(), ControlType.kPosition);
            previousIntakekP = changeableIntakekP.getAsDouble();   
        }

        // Update sensor readings
        inputs.rightIntakeWheelSpeedRPM = rightDoorMotor.getEncoder().getVelocity();
        inputs.leftIntakeWheelSpeedRPM = leftDoorMotor.getEncoder().getVelocity();
        inputs.currentDoorPosition = rightDoorMotor.getEncoder().getPosition();

    }
    
}
