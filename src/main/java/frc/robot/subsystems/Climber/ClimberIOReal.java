package frc.robot.subsystems.Climber;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ClimberConstants;

public class ClimberIOReal implements ClimberIO {
    private SparkMax climbMotor;
    private SparkClosedLoopController climbPID;

    private LoggedNetworkNumber kP = new LoggedNetworkNumber("Tuning/Shooter/kP", 0.0);
    private LoggedNetworkNumber kI = new LoggedNetworkNumber("Tuning/Shooter/kI", 0.0);
    private LoggedNetworkNumber kD = new LoggedNetworkNumber("Tuning/Shooter/kD", 0.0);

    private double lastkP;
    private double lastkI;
    private double lastkD;

    private boolean debugMode;


    public ClimberIOReal() {
        climbMotor = new SparkMax(ClimberConstants.CLIMBER_MOTOR_CAN_ID, MotorType.kBrushless);
        climbPID = climbMotor.getClosedLoopController();
        setInitialMotorPIDs();
        debugMode = true;
    }

    private void setInitialMotorPIDs() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
            .pid(0.0, 0.0, 0.0);
        config.closedLoop.feedForward
                .kS(0.0)
                .kV(0.0)
                .kA(0.0)
                .kG(0.0);
        config.closedLoop.maxMotion
            .cruiseVelocity(2000)
            .maxAcceleration(10000)
            .allowedProfileError(0.1);
        climbMotor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ClimberIOInputsAutoLogged inputs) {
        inputs.climberPosInRotations = climbMotor.getAbsoluteEncoder().getPosition();
    }

    @Override
    public void setClimbPosition(ClimbPosition position) {
        switch (position) {
            case NONE:
                climbPID.setSetpoint(ClimberConstants.CLIMBER_ZERO_POS, ControlType.kMAXMotionPositionControl);
                break;
            case AUTO:
                climbPID.setSetpoint(ClimberConstants.AUTO_CLIMB_POS, ControlType.kMAXMotionPositionControl);
                break;
            case RUNG_ONE:
                climbPID.setSetpoint(ClimberConstants.RUNG_ONE_CLIMBER_POS, ControlType.kMAXMotionPositionControl);
                break;
            case RUNG_TWO:
                climbPID.setSetpoint(ClimberConstants.RUNG_TWO_CLIMBER_POS, ControlType.kMAXMotionPositionControl);
                break;
            case RUNG_THREE:
                climbPID.setSetpoint(ClimberConstants.RUNG_THREE_CLIMBER_POS, ControlType.kMAXMotionPositionControl);
                break;
        }
    }

    public void tunePID() {
        if (debugMode) {
            if (kP.getAsDouble() != lastkP || kI.getAsDouble() != lastkI || kD.getAsDouble() != lastkD) {
                SparkMaxConfig config = new SparkMaxConfig();
            config.closedLoop
                .pid(kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble());
            config.closedLoop.feedForward
                    .kS(0.0)
                    .kV(0.0)
                    .kA(0.0)
                    .kG(0.0);
            config.closedLoop.maxMotion
                .cruiseVelocity(2000)
                .maxAcceleration(10000)
                .allowedProfileError(0.1);
            climbMotor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
            }
        }
    }


}