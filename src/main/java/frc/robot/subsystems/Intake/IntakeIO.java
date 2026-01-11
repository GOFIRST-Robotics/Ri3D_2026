package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs { 
        public boolean motorConnected = false; 

        /** Wheel speed in RPM */
        public double intakeWheelSpeedRPM = 0.0;
        public double intakeDoorPositionDegrees = 0.0;

        /** Belt speed in RPM */
        // public double intakeBeltSpeedRPM = 0.0;

        public double motorAppliedVolts = 0.0;
        public double motorCurrentAmps = 0.0; 

    }

    /** Updates the set of loggable inputs for Intake. */
    public default void updateInputs(IntakeIOInputs inputs) {}

    /** Run the Intake Belt/Wheels at a specified RPM (closed-loop). */
    public default void setIntakeWheelSpeedRPM(double speedRPM) {}

    /** Set the motor voltage directly (open-loop) */
    public default void setOpenLoopVolts(double volts) {}

    /** Stop all outputs. */
    public default void stop() { 
        setIntakeWheelSpeedRPM(0.0);
    }
    
}
