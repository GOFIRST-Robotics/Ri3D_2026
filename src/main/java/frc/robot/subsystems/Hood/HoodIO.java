package frc.robot.subsystems.Hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

    @AutoLog
    class HoodIOInputs {
        public double hoodRadians;
        public double motorPosition;

        public double targetRadians;
        public double targetPosition;
    }

    default void updateInputs(HoodIOInputs inputs) {}
    default void setHoodRadians(double radians) {}
    default void periodic() {}
}