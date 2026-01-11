package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputs inputs; 

    public Intake(IntakeIO io) {
        this.io = io;
        this.inputs = new IntakeIOInputs();
    }

    public void runIntake(double RPM) {
        io.setIntakeWheelSpeedRPM(RPM);
    }

    public void setIntakeDoorPosition(double position) {
        io.setIntakeDoorPosition(position, inputs);
    }

    
}
