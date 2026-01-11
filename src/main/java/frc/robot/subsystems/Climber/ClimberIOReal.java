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

    private LoggedNetworkNumber changablekP = new LoggedNetworkNumber("Tuning/Climber/kP", 0.0);
    private LoggedNetworkNumber changablekI = new LoggedNetworkNumber("Tuning/Climber/kI", 0.0);
    private LoggedNetworkNumber changablekD = new LoggedNetworkNumber("Tuning/Climber/kD", 0.0);

    private double climbMotorkP;
    private double climbMotorkI;
    private double climbMotorkD;

    public ClimberIOReal() {
        climbMotor = new SparkMax(ClimberConstants.CLIMBER_MOTOR_CAN_ID, MotorType.kBrushless);
        climbPID = climbMotor.getClosedLoopController();
        setInitialMotorSettings();
    }

    private void setInitialMotorSettings() {
        climbMotorkP = 0.1;
        climbMotorkI = 0.0;
        climbMotorkD = 0.0;
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
            .pid(climbMotorkP, climbMotorkI, climbMotorkD);
        config.closedLoop.feedForward
                .kS(0.0)
                .kV(0.0)
                .kA(0.0)
                .kG(0.0);
        config.closedLoop.maxMotion
            .cruiseVelocity(2000)
            .maxAcceleration(4000)
            .allowedProfileError(0.5);
        config.softLimit
            .forwardSoftLimit(50)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(0.5)
            .reverseSoftLimitEnabled(true);
        config.inverted(false);
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

    @Override
    public void tunePID() {
        if (changablekP.getAsDouble() != climbMotorkP || changablekI.getAsDouble() != climbMotorkI || changablekD.getAsDouble() != climbMotorkD) {
            SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
            .pid(changablekP.getAsDouble(), changablekI.getAsDouble(), changablekD.getAsDouble());
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