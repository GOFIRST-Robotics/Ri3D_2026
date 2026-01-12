package frc.robot.subsystems.Flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    /////////////////////////////////////////////////////
    // TODO: MAKE METHOD TO DETERMINE WHEN AT SPEED!!! //
    /////////////////////////////////////////////////////

    public Flywheel(FlywheelIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
    }

    public void runFlywheels(double topRPM, double bottomRPM)
    {
        io.setTopFlywheelRPM(topRPM);
        io.setBottomFlywheelRPM(bottomRPM);
    }

    public void setLaunchSpeed(double launchSpeed)
    {
        double rpm = (60 / Math.PI) * (launchSpeed / Constants.WHEEL_DIAMETER);

        runFlywheels(rpm, rpm);
    }

    public Command RunFlywheelsCommand() { return this.runOnce(() -> runFlywheels(1200, 1200)); }

    public Command StopFlywheelsCommand() { return this.runOnce(() -> runFlywheels(0, 0)); }
}