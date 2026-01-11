package frc.robot.subsystems.Intake;

import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.robot.Constants.IntakeConstants;






public class IntakeIOSpark implements IntakeIO {
    private final SparkMax rightDoorMotor;
    private final SparkMax leftDoorMotor;
    private final VictorSP intakeWheel;
    
    private double intakeDoorKp;
    private LoggedNetworkNumber changeableIntakekP;

    public IntakeIOSpark() { 
        rightDoorMotor = new SparkMax(IntakeConstants.INTAKE_UPPER_DOOR_MOTOR_ID, MotorType.kBrushless); 
        leftDoorMotor = new SparkMax(IntakeConstants.INTAKE_LOWER_DOOR_MOTOR_ID, MotorType.kBrushless);
        intakeWheel = new VictorSP(IntakeConstants.INTAKE_WHEEL_MOTOR_ID); 

        var doorConfig = new SparkMaxConfig();
        var followerConfig = new SparkMaxConfig();

        doorConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pidf(IntakeConstants.INTAKE_DOOR_kP, 0.0, 0.0, 0.0);

        followerConfig.follow(rightDoorMotor);
        followerConfig.inverted(true);
        leftDoorMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        setIntakeDoorPlacement();
    }   


    private void setIntakeDoorPlacement() {
        intakeDoorKp = IntakeConstants.INTAKE_DOOR_kP;
        changeableIntakekP = new LoggedNetworkNumber("Intake/ChangeableIntakeKP", intakeDoorKp);

        SparkMaxConfig doorConfig = new SparkMaxConfig();

        doorConfig.closedLoop.pid(intakeDoorKp, 0.0, 0.0);
        
        rightDoorMotor.configure(doorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    @Override
    public void setIntakeWheelSpeedRPM(double RPM) {
        intakeWheel.set(RPM);
    }

    public void setIntakeDoorPosition(double position, IntakeIOInputs inputs) {
        inputs.desiredDoorPosition = position;
        rightDoorMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) { 
        inputs.rightIntakeWheelSpeedRPM = rightDoorMotor.getEncoder().getVelocity();
        inputs.leftIntakeWheelSpeedRPM = leftDoorMotor.getEncoder().getVelocity();
        inputs.currentDoorPosition = rightDoorMotor.getEncoder().getPosition();
    }
        


    


    
}
