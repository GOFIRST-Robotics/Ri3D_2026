package frc.robot.subsystems.Indexer;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.IndexerConstants;

public class IndexerIOReal implements IndexerIO {
    private SparkMax indexerMotor;
    private SparkMax elevatorMotor;
    private SparkClosedLoopController indexerPID;
    private SparkClosedLoopController elevatorPID;

    private LoggedNetworkNumber changableIndexerkP;
    private LoggedNetworkNumber changableIndexerkFF;
    private LoggedNetworkNumber changableElevatorkP;
    private LoggedNetworkNumber changableElevatorkFF;

    private double indexerkP;
    private double indexerkFF;
    private double elevatorkP;
    private double elevatorkFF;

    public IndexerIOReal() {
        indexerMotor = new SparkMax(IndexerConstants.INDEXER_MOTOR_CAN_ID, MotorType.kBrushless);
        elevatorMotor = new SparkMax(IndexerConstants.ELEVATOR_MOTOR_CAN_ID, MotorType.kBrushless);
        indexerPID = indexerMotor.getClosedLoopController();
        elevatorPID = elevatorMotor.getClosedLoopController();
        setInitialMotorSettings();
    }

    private void setInitialMotorSettings() {
        indexerkP = IndexerConstants.INDEXER_MOTOR_kP;
        indexerkFF = IndexerConstants.INDEXER_MOTOR_kFF;
        elevatorkP = IndexerConstants.ELEVATOR_MOTOR_kP;
        elevatorkFF = IndexerConstants.ELEVATOR_MOTOR_kFF;
        changableIndexerkP = new LoggedNetworkNumber("Tuning/Indexer/IndexerkP", indexerkP);
        changableIndexerkFF = new LoggedNetworkNumber("Tuning/Indexer/IndexerkFF", indexerkFF);
        changableElevatorkP = new LoggedNetworkNumber("Tuning/Elevator/kP", elevatorkP);
        changableElevatorkFF = new LoggedNetworkNumber("Tuning/Elevator/kFF", elevatorkFF);
        SparkMaxConfig indexerConfig = new SparkMaxConfig();
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        indexerConfig.closedLoop.pidf(indexerkP, 0.0, 0.0, indexerkFF);
        elevatorConfig.closedLoop.pidf(elevatorkP, 0.0, 0.0, elevatorkFF);
        indexerMotor.configure(indexerConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        elevatorMotor.configure(elevatorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IndexerIOInputsAutoLogged inputs) {
        inputs.indexerMotorRotationalPos = indexerMotor.getAbsoluteEncoder().getPosition();
        inputs.indexerMotorVelRPM = indexerMotor.getAbsoluteEncoder().getVelocity();
        inputs.elevatorMotorVelRPM = elevatorMotor.getAbsoluteEncoder().getVelocity();
    }

    @Override
    public void setIndexerRPM(double RPM) {
        indexerPID.setSetpoint(RPM, ControlType.kVelocity);
    }

    @Override
    public void setElevatorRPM(double RPM) {
        elevatorPID.setSetpoint(RPM, ControlType.kVelocity);
    }

    @Override
    public void tunePID() {
        if (changableIndexerkP.getAsDouble() != indexerkP || changableIndexerkFF.getAsDouble() != indexerkFF) {
            SparkMaxConfig indexerConfig = new SparkMaxConfig();
            indexerkP = changableElevatorkP.getAsDouble();
            indexerkFF = changableElevatorkFF.getAsDouble();
            indexerConfig.closedLoop.pidf(indexerkP, 0.0, 0.0, indexerkFF);
            indexerMotor.configure(indexerConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        }
        if (changableElevatorkP.getAsDouble() != elevatorkP || changableElevatorkFF.getAsDouble() != elevatorkFF) {
            SparkMaxConfig elevatorConfig = new SparkMaxConfig();
            elevatorkP = changableElevatorkP.getAsDouble();
            elevatorkFF = changableElevatorkFF.getAsDouble();
            elevatorConfig.closedLoop.pidf(elevatorkP, 0.0, 0.0, elevatorkFF);
            elevatorMotor.configure(elevatorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        }
        
    }



}
