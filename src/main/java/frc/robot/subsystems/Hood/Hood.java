package frc.robot.subsystems.Hood;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;

public class Hood extends SubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    // Dependency Injection: We pass the IO in, we don't create it here.
    public Hood(HoodIO io) {
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
        if (currentTargetRadians < Constants.TurretConstants.TURRET_HOOD_MIN_RADIANS) { currentTargetRadians = Constants.TurretConstants.TURRET_HOOD_MIN_RADIANS; }
        else if (currentTargetRadians > Constants.TurretConstants.TURRET_HOOD_MAX_RADIANS) { currentTargetRadians = Constants.TurretConstants.TURRET_HOOD_MAX_RADIANS; }

        io.setHoodRadians(currentTargetRadians); 
    }

    public Command IncrementHoodAngleCommand() { return this.run(() -> setTargetRadians(currentTargetRadians + Constants.TurretConstants.TURRET_HOOD_CHANGE_SPEED)); }
    public Command DecrementHoodAngleCommand() { return this.run(() -> setTargetRadians(currentTargetRadians - Constants.TurretConstants.TURRET_HOOD_CHANGE_SPEED)); }
}