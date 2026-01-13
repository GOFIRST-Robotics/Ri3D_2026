package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputs inputs; 

    public Intake(IntakeIO io) {
        this.io = io;
        this.inputs = new IntakeIOInputs();
    }

    public void runIntake(double percentOutput) {
        io.setIntakeWheelSpeedPercentOut(percentOutput);
    }

    public void MoveIntakeToPosition(double position) {
        io.setIntakeDoorPosition(position, inputs);
    }

    public void stopIntake() {
        io.stop(inputs);
    }



    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Command setIntake(double position) {
        return this.runOnce(()->MoveIntakeToPosition(position));
    }




        /** Run intake wheels outward (eject game piece) */
    public Command ejectCommand() {
        return this.startEnd(
            () -> runIntake(-IntakeConstants.INTAKE_WHEEL_SPEED),
            () -> runIntake(0)
        );
    }

    /** Run intake wheels at custom speed (while held) */
    public Command runWheelsCommand(double percentOutput) {
        return this.startEnd(
            () -> runIntake(percentOutput),
            () -> runIntake(0)
        );
    }

    /** Stop all intake motors */
    public Command stopCommand() {
        return this.runOnce(() -> stopIntake());
    }


        /** Move door to deployed/open position */
    public Command deployDoorCommand() {
        return this.runOnce(() -> MoveIntakeToPosition(IntakeConstants.INTAKE_DOOR_POSITION_DEPLOYED));
    }

    /** Move door to stowed/closed position */
    public Command stowDoorCommand() {
        return this.runOnce(() -> MoveIntakeToPosition(IntakeConstants.INTAKE_DOOR_POSITION_STORED));
    }

    /** Run intake wheels inward (intake game piece) */
    public Command intakeCommand() {
        return this.startEnd(
            () -> runIntake(IntakeConstants.INTAKE_WHEEL_SPEED),
            () -> runIntake(0)
        );
    }

        /** Deploy door and run intake wheels (full intake sequence) */
    public Command fullIntakeCommand() {
        return deployDoorCommand()
            .andThen(intakeCommand());
    }

    /** Stop wheels and stow door (full stow sequence) */
    public Command fullStowCommand() {
        return this.runOnce(() -> runIntake(0))
            .andThen(stowDoorCommand());
    }

    /** Deploy, intake, then stow when released */
    public Command intakeAndStowCommand() {
        return this.startEnd(
            () -> {
                MoveIntakeToPosition(IntakeConstants.INTAKE_DOOR_POSITION_DEPLOYED);
                runIntake(IntakeConstants.INTAKE_WHEEL_SPEED);
            },
            () -> {
                runIntake(0);
                MoveIntakeToPosition(IntakeConstants.INTAKE_DOOR_POSITION_STORED);
            }
        );
    }

    
}
