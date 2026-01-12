package frc.robot.subsystems.Flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

    @AutoLog
    class FlywheelIOInputs {
        public double topFlywheelRPM;
        public double bottomFlywheelRPM;
    }

    default void updateInputs(FlywheelIOInputs inputs) {}
    default void setTopFlywheelRPM(double rpm) {}
    default void setBottomFlywheelRPM(double rpm) {}
    default void tuneTopAndBottomRPM() {}
    default void periodic() {}
}