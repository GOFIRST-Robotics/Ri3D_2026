package frc.robot.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Flywheel.Flywheel;
import frc.robot.subsystems.Hood.Hood;

public class Turret extends SubsystemBase {

    private final Flywheel flywheel;
    private final Hood hood;

    // Dependency Injection: We pass the IO in, we don't create it here.
    public Turret(Flywheel flywheel, Hood hood) {
        this.flywheel = flywheel;
        this.hood = hood;
    }

    @Override
    public void periodic() {

    }
}