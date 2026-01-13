package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputs inputs; 

    public Intake(IntakeIO io) {
        this.io = io;
        this.inputs = new IntakeIOInputs();
        setIntakeDoorPosition(IntakeConstants.INTAKE_DOOR_POSITION_STORED);
    }

    public void runIntake(double percentOutput) {
        io.setIntakeWheelSpeedPercentOut(percentOutput);
    }

    public void setIntakeDoorPosition(double position) {
        io.setIntakeDoorPosition(position, inputs);
    }

    public void stopIntake() {
        io.stop();
    }



    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.stop();
    }


    
}
