package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;


public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {  

        /** Wheel speed in RPM */
        public double rightIntakeWheelSpeedRPM = 0.0; 
        public double leftIntakeWheelSpeedRPM = 0.0;

        /** Door position in degrees */
        public double currentDoorPosition = 0.0;
        public double desiredDoorPosition = 0.0;

    }

    /** Run the Intake Belt/Wheels at a specified RPM (closed-loop). */
    public default void setIntakeWheelSpeedPercentOut(double percentOutput) {}

    /** Set the motor voltage directly (open-loop) */
    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setIntakeDoorPosition(double position, IntakeIOInputs inputs) {}
    


    /** Stop all outputs. */
    public default void stop() { 
        setIntakeWheelSpeedPercentOut(0.0);
    }
    
}
