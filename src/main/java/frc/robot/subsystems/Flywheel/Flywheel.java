package frc.robot.subsystems.Flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    // Dependency Injection: We pass the IO in, we don't create it here.
    public Flywheel(FlywheelIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // Update the inputs from Real Hardware or Sim
        io.updateInputs(inputs);
        // Log the data (AdvantageKit style)
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

        runFlywheels(rpm, -rpm);
    }
}