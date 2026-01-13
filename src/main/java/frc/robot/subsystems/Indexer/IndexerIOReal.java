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
    private SparkClosedLoopController indexerPID;

    private LoggedNetworkNumber changableIndexerkP;
    private LoggedNetworkNumber changableIndexerkFF;

    private double indexerkP;
    private double indexerkV;

    public IndexerIOReal() {
        indexerMotor = new SparkMax(IndexerConstants.INDEXER_MOTOR_CAN_ID, MotorType.kBrushless);
        indexerPID = indexerMotor.getClosedLoopController();
        setInitialMotorSettings();
    }

    private void setInitialMotorSettings() {
        indexerkP = IndexerConstants.INDEXER_MOTOR_kP;
        indexerkV = IndexerConstants.INDEXER_MOTOR_kV;
        changableIndexerkP = new LoggedNetworkNumber("Tuning/Indexer/IndexerkP", indexerkP);
        changableIndexerkFF = new LoggedNetworkNumber("Tuning/Indexer/IndexerkFF", indexerkV);
        SparkMaxConfig indexerConfig = new SparkMaxConfig();
        indexerConfig.closedLoop.pid(indexerkP, 0.0, 0.0);
        indexerConfig.closedLoop.feedForward
        .kV(indexerkV);
        indexerConfig.closedLoop.maxMotion
        .cruiseVelocity(IndexerConstants.INDEXER_MOTOR_MAX_VEL)
        .maxAcceleration(IndexerConstants.INDEXER_MOTOR_MAX_ACCEL);
        indexerConfig.inverted(true);
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
    public void setIndexerKDutyCycle(double percentOutput) {
        indexerPID.setSetpoint(percentOutput, ControlType.kDutyCycle);
    }

    @Override
    public void periodic() {
        if (changableIndexerkP.getAsDouble() != indexerkP || changableIndexerkFF.getAsDouble() != indexerkV) {
            SparkMaxConfig indexerConfig = new SparkMaxConfig();
            indexerkP = changableIndexerkP.getAsDouble();
            indexerkV = changableIndexerkFF.getAsDouble();
            indexerConfig.closedLoop.pid(indexerkP, 0.0, 0.0);
            indexerConfig.closedLoop.feedForward
            .kV(indexerkV);
            indexerMotor.configure(indexerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }
    }



}
