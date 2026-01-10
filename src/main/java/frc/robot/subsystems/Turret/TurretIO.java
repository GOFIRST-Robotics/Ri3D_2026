package frc.robot.subsystems.Turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

    @AutoLog
    class TurretIOInputs {
        public double turretRadians;
        public double hoodRadians;
        public double leftFlywheelRPM;
        public double rightFlywheelRPM;
    }

    default void updateInputs(TurretIOInputs inputs) {}
    default void setTurretRadians(double radians) {}
    default void setHoodRadians(double radians) {}
    default void setLeftFlywheelRPM(double rpm) {}
    default void setRightFlywheelRPM(double rpm) {}
}