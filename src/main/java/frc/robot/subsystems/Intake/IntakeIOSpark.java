package frc.robot.subsystems.Intake;

import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.robot.Constants.IntakeConstants;






public class IntakeIOSpark implements IntakeIO {
    private final SparkMax rightDoorMotor;
    private final SparkMax leftDoorMotor;
    private final VictorSP intakeWheel;
    
    private LoggedNetworkNumber changeableIntakekP;
    private double previousIntakekP = IntakeConstants.INTAKE_DOOR_kP;

    public IntakeIOSpark() { 
        rightDoorMotor = new SparkMax(IntakeConstants.INTAKE_UPPER_DOOR_MOTOR_ID, MotorType.kBrushless); 
        leftDoorMotor = new SparkMax(IntakeConstants.INTAKE_LOWER_DOOR_MOTOR_ID, MotorType.kBrushless);
        intakeWheel = new VictorSP(IntakeConstants.INTAKE_WHEEL_MOTOR_ID); 

        changeableIntakekP = new LoggedNetworkNumber("Intake/ChangeableIntakeKP", IntakeConstants.INTAKE_DOOR_kP);


        

        var doorConfig = new SparkMaxConfig();
        doorConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(changeableIntakekP.getAsDouble(), 0.0, 0.0);
        doorConfig.absoluteEncoder.positionConversionFactor(2 * Math.PI); // Radians to degrees
        rightDoorMotor.configure(doorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        var followerConfig = new SparkMaxConfig();
        followerConfig.follow(rightDoorMotor);
        followerConfig.inverted(true);
        leftDoorMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }   

    @Override
    public void setIntakeWheelSpeedPercentOut(double percentOutput) {
        intakeWheel.set(percentOutput);
    }

    public void setIntakeDoorPosition(double position, IntakeIOInputs inputs) {
        inputs.desiredDoorPosition = Units.degreesToRadians(position);;
        rightDoorMotor.getClosedLoopController().setSetpoint(position, ControlType.kPosition);
    
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) { 
        if(changeableIntakekP.getAsDouble() != previousIntakekP) {
            rightDoorMotor.getClosedLoopController().setSetpoint(changeableIntakekP.getAsDouble(), ControlType.kPosition);
            previousIntakekP = changeableIntakekP.getAsDouble();   
        }
        
        inputs.rightIntakeWheelSpeedRPM = rightDoorMotor.getEncoder().getVelocity();
        inputs.leftIntakeWheelSpeedRPM = leftDoorMotor.getEncoder().getVelocity();
        inputs.currentDoorPosition = rightDoorMotor.getEncoder().getPosition();

        

    }
        


    


    
}
