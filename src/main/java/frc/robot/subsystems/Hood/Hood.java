package frc.robot.subsystems.Hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;

public class Hood extends SubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public Hood(HoodIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
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

    public Command incrementHoodAngleCommand() { return this.run(() -> setTargetRadians(currentTargetRadians + Constants.TurretConstants.TURRET_HOOD_CHANGE_SPEED)); }
    public Command decrementHoodAngleCommand() { return this.run(() -> setTargetRadians(currentTargetRadians - Constants.TurretConstants.TURRET_HOOD_CHANGE_SPEED)); }

    public boolean HoodRotationWithinError()
    {
        return Math.abs(inputs.hoodRadians - currentTargetRadians) <= Constants.TurretConstants.TURRET_HOOD_ACCEPTABLE_RADIAN_ERROR;
    }
}