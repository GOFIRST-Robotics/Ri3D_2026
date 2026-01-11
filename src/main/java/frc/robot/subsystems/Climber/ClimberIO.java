package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

    @AutoLog
    class ClimberIOInputs {
        public double climberPosInRotations;
    }

    enum ClimbPosition {
        NONE,
        AUTO,
        RUNG_ONE,
        RUNG_TWO,
        RUNG_THREE
    }

    default void updateInputs(ClimberIOInputsAutoLogged inputs) {}

    default void setClimbPosition(ClimbPosition position) {}

    default void tunePID() {}
}
