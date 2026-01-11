package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    @AutoLog
    class IndexerIOInputs {
        public double indexerMotorRotationalPos;
        public double indexerMotorVelRPM;
        public double elevatorMotorVelRPM;
    }

    default void updateInputs(IndexerIOInputsAutoLogged inputs) {}
    default void setIndexerRPM(double RPM) {}
    default void setElevatorRPM(double RPM) {}
    default void periodic() {}
}
