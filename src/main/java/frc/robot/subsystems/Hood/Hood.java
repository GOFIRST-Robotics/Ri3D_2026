package frc.robot.subsystems.Hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

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

    public void setTargetRadians(double radians)
    {
        io.setHoodRadians(radians);
    }
}