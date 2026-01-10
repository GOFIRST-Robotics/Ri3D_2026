package frc.robot.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    // Dependency Injection: We pass the IO in, we don't create it here.
    public Turret(TurretIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // Update the inputs from Real Hardware or Sim
        io.updateInputs(inputs);
        // Log the data (AdvantageKit style)
        Logger.processInputs("Turret", inputs);
    }
}