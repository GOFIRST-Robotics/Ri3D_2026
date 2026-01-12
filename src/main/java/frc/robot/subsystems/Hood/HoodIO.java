package frc.robot.subsystems.Hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

    @AutoLog
    class HoodIOInputs {
        public double hoodRadians;
    }

    default void updateInputs(HoodIOInputs inputs) {}
    default void setHoodRadians(double radians) {}
    default void periodic() {}
}