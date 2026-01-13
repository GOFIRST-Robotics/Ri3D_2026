package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    @AutoLog
    class IndexerIOInputs {
        public double indexerMotorRotationalPos;
        public double indexerMotorVelRPM;
    }

    default void updateInputs(IndexerIOInputsAutoLogged inputs) {}
    default void setIndexerRPM(double RPM) {}
    default void setIndexerKDutyCycle(double percentOutput) {}
    default void periodic() {}
}
