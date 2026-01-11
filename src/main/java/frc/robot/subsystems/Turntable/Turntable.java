package frc.robot.subsystems.Turntable;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    public void setTargetRadians(double radians)
    {
        io.setTurntableRadians(radians);
    }
}