package frc.robot.subsystems.Turntable;

import org.littletonrobotics.junction.AutoLog;

public interface TurntableIO {

    @AutoLog
    class TurntableIOInputs {
        public double turntableRadians;
        public double motorPosition;
        public double targetTurntableRadians;
        public double targetMotorPosition;
    }

    default void updateInputs(TurntableIOInputs inputs) {}
    default void setTurntableRadians(double radians) {}
    default void periodic() {}
}