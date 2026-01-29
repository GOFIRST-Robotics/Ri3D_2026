package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;


public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {  

        /** Wheel speed in RPM */
        public double rightIntakeWheelSpeedRPM = 0.0; 
        public double leftIntakeWheelSpeedRPM = 0.0;

        /** Door position in radians */
        public double currentDoorPosition = 0.0;
        public double desiredDoorPosition = 0.0;

    }

    /** Run the Intake Belt/Wheels at a specified percent output (open-loop). */
    public default void setIntakeWheelSpeedPercentOut(double percentOutput) {}

    /** Update sensor inputs */
    public default void updateInputs(IntakeIOInputs inputs) {}

    /** Set the door to a target position (closed-loop control) */
    public default void setIntakeDoorPosition(double position, IntakeIOInputs inputs) {}

    /** Set door motor speed directly (open-loop control) */
    public default void setDoorSpeed(double percentOutput) {}

    /** Reset safety trip flag to re-enable the intake */
    public default void resetSafety() {}

    public default boolean isDown() { return false; }
    public default boolean setPointZero() { return false; }

    public default void setkDutyZero() {}

    /** Stop all outputs. */
    public default void stop(IntakeIOInputs inputs) { 
        setIntakeWheelSpeedPercentOut(0.0);
        setDoorSpeed(0.0);
    }
}