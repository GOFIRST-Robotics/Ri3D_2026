package frc.robot.subsystems.Flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

    @AutoLog
    class FlywheelIOInputs {
        public double leftFlywheelRPM;
        public double rightFlywheelRPM;
    }

    default void updateInputs(FlywheelIOInputs inputs) {}
    default void setLeftFlywheelRPM(double rpm) {}
    default void setRightFlywheelRPM(double rpm) {}
}