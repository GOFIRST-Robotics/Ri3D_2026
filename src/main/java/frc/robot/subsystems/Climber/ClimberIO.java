package frc.robot.subsystems.Climber;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

    @AutoLog
    class ClimberIOInputs {
        public double climberPosInRotations;
    }

    enum ClimbPosition {
        ZERO,
        AUTO,
        RUNG_ONE,
        RUNG_TWO,
        RUNG_THREE
    }

    default void updateInputs(ClimberIOInputsAutoLogged inputs) {}

    default void setClimbExtentionPos(ClimbPosition position) {}

    default void setClimbRetractionPos(ClimbPosition position) {}

    default void setServoOnePos(double position) {}

    default void setServoTwoPos(double position) {}

    default boolean isMovementFinished() {return false;}

    default void periodic() {}
}
