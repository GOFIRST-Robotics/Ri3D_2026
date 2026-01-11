package frc.robot.subsystems.Indexer;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.robot.Constants.IndexerConstants;

public class IndexerIOReal implements IndexerIO {
    private SparkMax indexerMotor;
    private VictorSP elevatorMotor;
    private SparkClosedLoopController indexerPID;

    private LoggedNetworkNumber changableIndexerkP;
    private LoggedNetworkNumber changableIndexerkFF;

    private double indexerkP;
    private double indexerkFF;

    public IndexerIOReal() {
        indexerMotor = new SparkMax(IndexerConstants.INDEXER_MOTOR_CAN_ID, MotorType.kBrushless);
        elevatorMotor = new VictorSP(IndexerConstants.ELEVATOR_MOTOR_CAN_ID);
        indexerPID = indexerMotor.getClosedLoopController();
        setInitialMotorSettings();
    }

    private void setInitialMotorSettings() {
        indexerkP = IndexerConstants.INDEXER_MOTOR_kP;
        indexerkFF = IndexerConstants.INDEXER_MOTOR_kFF;
        changableIndexerkP = new LoggedNetworkNumber("Tuning/Indexer/IndexerkP", indexerkP);
        changableIndexerkFF = new LoggedNetworkNumber("Tuning/Indexer/IndexerkFF", indexerkFF);
        SparkMaxConfig indexerConfig = new SparkMaxConfig();
        indexerConfig.closedLoop.pid(indexerkP, 0.0, 0.0);
        indexerConfig.closedLoop.feedForward
        .kV(indexerkFF);
        indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IndexerIOInputsAutoLogged inputs) {
        inputs.indexerMotorRotationalPos = indexerMotor.getEncoder().getPosition();
        inputs.indexerMotorVelRPM = indexerMotor.getEncoder().getVelocity();
    }

    @Override
    public void setIndexerRPM(double RPM) {
        indexerPID.setSetpoint(RPM, ControlType.kVelocity);
    }

    @Override
    public void setElevatorRPM(double RPM) {
        elevatorMotor.set(RPM);
    }

    @Override
    public void periodic() {
        if (changableIndexerkP.getAsDouble() != indexerkP || changableIndexerkFF.getAsDouble() != indexerkFF) {
            SparkMaxConfig indexerConfig = new SparkMaxConfig();
            indexerkP = changableIndexerkP.getAsDouble();
            indexerkFF = changableIndexerkFF.getAsDouble();
            indexerConfig.closedLoop.pid(indexerkP, 0.0, 0.0);
            indexerConfig.closedLoop.feedForward
            .kV(indexerkFF);
            indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }



}
