package frc.robot.subsystems.Intake;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;



public class IntakeIOSpark implements IntakeIO {
    private final SparkBase spark;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;

    public IntakeIOSpark() { 
        spark = new SparkMax(10, MotorType.kBrushed); // Example CAN ID
        encoder = spark.getEncoder();
        controller = spark.getClosedLoopController();

        var config = new SparkMaxConfig();

        config
            .closedLoop
            .feedbackSensor(/** Feedback Val */)
            .pidf(/** PIDF Values */);


        //hmm idk yet
        // tryUntilOk(
        //     spark, 
        //     5, 
        //     () -> spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));


    }   

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

        

        ifOk(spark, encoder::getVelocity, (value) -> inputs.intakeWheelSpeedRPM = value);
        ifOk(spark, spark::getOutputCurrent, (value) -> inputs.motorCurrentAmps = value);

    }

    @Override 
    public void setOpenLoopVolts(double volts) {
        spark.setVoltage(volts);
    }

    @Override
    public void setIntakeWheelSpeedRPM(double speedRPM) {
        controller.setReference(
            speedRPM,
            ControlType.kVelocity,
            0,
            0.0,
            ArbFFUnits.kRPM);
    }



    
}
