package frc.robot.subsystems.Turntable;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class Turntable extends SubsystemBase {
    private final TurntableIO io;
    private final TurntableIOInputsAutoLogged inputs = new TurntableIOInputsAutoLogged();

    // Dependency Injection: We pass the IO in, we don't create it here.
    public Turntable(TurntableIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // Update the inputs from Real Hardware or Sim
        io.updateInputs(inputs);
        // Log the data (AdvantageKit style)
        Logger.processInputs("Hood", inputs);
    }

    private double currentTargetRadians;
    public void setTargetRadians(double radians) 
    { 
        currentTargetRadians = radians;
        if (currentTargetRadians < -Constants.TurretConstants.TURRET_TURNTABLE_MAX_RADIANS) { currentTargetRadians = -Constants.TurretConstants.TURRET_TURNTABLE_MAX_RADIANS; }
        else if (currentTargetRadians > Constants.TurretConstants.TURRET_TURNTABLE_MAX_RADIANS) { currentTargetRadians = Constants.TurretConstants.TURRET_TURNTABLE_MAX_RADIANS; }

        io.setTurntableRadians(currentTargetRadians); 
    }

    public Command IncrementTurntableAngleCommand() { return this.run(() -> setTargetRadians(currentTargetRadians + Constants.TurretConstants.TURRET_TURNTABLE_CHANGE_SPEED)); }
    public Command DecrementTurntableAngleCommand() { return this.run(() -> setTargetRadians(currentTargetRadians - Constants.TurretConstants.TURRET_TURNTABLE_CHANGE_SPEED)); }
}