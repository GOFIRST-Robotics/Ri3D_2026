package frc.robot.subsystems.Hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.subsystems.Turret.Turret;

public class Hood extends SubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private final double MAX_RADIANS = Constants.TurretConstants.TURRET_HOOD_MAX_RADIANS - Constants.TurretConstants.TURRET_HOOD_MIN_RADIANS;

    public Hood(HoodIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        inputs.targetRadians = currentTargetRadians;
        
        Logger.processInputs("Hood", inputs);
    }

    private double currentTargetRadians = 0; // assuming the hood starts fully down
    public void setTargetRadians(double radians) 
    { 
        currentTargetRadians = radians;
        if (currentTargetRadians < 0) { currentTargetRadians = 0; }
        else if (currentTargetRadians > MAX_RADIANS) { currentTargetRadians = MAX_RADIANS; }

        io.setHoodRadians(currentTargetRadians); 
    }

    public void setDesiredLaunchAngle(double radians)
    {
        setTargetRadians(Constants.TurretConstants.TURRET_HOOD_MAX_RADIANS - radians);
    }

    public Command incrementHoodAngleCommand() { return this.run(() -> setTargetRadians(currentTargetRadians + Constants.TurretConstants.TURRET_HOOD_CHANGE_SPEED)); }
    public Command decrementHoodAngleCommand() { return this.run(() -> setTargetRadians(currentTargetRadians - Constants.TurretConstants.TURRET_HOOD_CHANGE_SPEED)); }
    public Command setHoodAngleCommand(double angle) { return this.runOnce(() -> setTargetRadians(angle)); }

    public boolean HoodRotationWithinError()
    {
        return Math.abs(inputs.hoodRadians - currentTargetRadians) <= Constants.TurretConstants.TURRET_HOOD_ACCEPTABLE_RADIAN_ERROR;
    }
}