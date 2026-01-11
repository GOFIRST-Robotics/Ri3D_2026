package frc.robot.subsystems.Climber;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ClimberConstants;

public class ClimberIOReal implements ClimberIO {
    private SparkMax climbMotor;
    private SparkClosedLoopController climbPID;

    private LoggedNetworkNumber changablekP;
    private LoggedNetworkNumber changablekI;
    private LoggedNetworkNumber changablekD;
    private LoggedNetworkNumber changablekV;
    private LoggedNetworkNumber changablekG;
    private LoggedNetworkNumber changableCruiseVel;
    private LoggedNetworkNumber changableMaxAccel;

    private double climbMotorkP;
    private double climbMotorkI;
    private double climbMotorkD;
    private double climbMotorkV;
    private double climbMotorkG;
    private double climbMotorCruiseVel;
    private double climbMotorMaxAccel;

    public ClimberIOReal() {
        climbMotor = new SparkMax(ClimberConstants.CLIMBER_MOTOR_CAN_ID, MotorType.kBrushless);
        climbPID = climbMotor.getClosedLoopController();
        setInitialMotorSettings();
    }

    private void setInitialMotorSettings() {
        climbMotorkP = ClimberConstants.CLIMB_MOTOR_KP;
        climbMotorkI = ClimberConstants.CLIMB_MOTOR_KI;
        climbMotorkD = ClimberConstants.CLIMB_MOTOR_KD;
        climbMotorkV = ClimberConstants.CLIMB_MOTOR_KV;
        climbMotorkG = ClimberConstants.CLIMB_MOTOR_KG;
        climbMotorCruiseVel = ClimberConstants.CLIMB_MOTOR_CRUISE_VEL;
        climbMotorMaxAccel = ClimberConstants.CLIMB_MOTOR_MAX_ACCEL;

        changablekP = new LoggedNetworkNumber("Tuning/Climber/kP", climbMotorkP);
        changablekI = new LoggedNetworkNumber("Tuning/Climber/kI", climbMotorkI);
        changablekD = new LoggedNetworkNumber("Tuning/Climber/kD", climbMotorkD);
        changablekV = new LoggedNetworkNumber("Tuning/Climber/kV", climbMotorkV);
        changablekG = new LoggedNetworkNumber("Tuning/Climber/kG", climbMotorkG);
        changableCruiseVel = new LoggedNetworkNumber("Tuning/Climber/CruiseVel", climbMotorCruiseVel);
        changableMaxAccel = new LoggedNetworkNumber("Tuning/Climber/MaxAccel", climbMotorMaxAccel);

        climbMotor.getEncoder().setPosition(0);

        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop
            .pid(climbMotorkP, climbMotorkI, climbMotorkD);
        config.closedLoop.feedForward
                .kS(0.0)
                .kV(climbMotorkV)
                .kA(0.0)
                .kG(climbMotorkG);
        config.closedLoop.maxMotion
            .cruiseVelocity(climbMotorCruiseVel)
            .maxAcceleration(climbMotorMaxAccel)
            .allowedProfileError(ClimberConstants.CLIMB_ALLOWED_ERROR);
        config.softLimit
            .forwardSoftLimit(ClimberConstants.CLIMB_MAX_POSITION_ALLOWED)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(ClimberConstants.CLIMB_MIN_POSITION_ALLOWED)
            .reverseSoftLimitEnabled(true);
        config.inverted(false);
        climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ClimberIOInputsAutoLogged inputs) {
        inputs.climberPosInRotations = climbMotor.getEncoder().getPosition();
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
    public boolean isMovementFinished() {
        return Math.abs(climbPID.getSetpoint() - climbMotor.getEncoder().getPosition()) < ClimberConstants.CLIMB_ALLOWED_ERROR;
    }

    @Override
    public void periodic() {
        boolean newConfig = false;
        SparkMaxConfig config = new SparkMaxConfig();
        if (changablekP.getAsDouble() != climbMotorkP || changablekI.getAsDouble() != climbMotorkI || changablekD.getAsDouble() != climbMotorkD) {
            climbMotorkP = changablekP.getAsDouble();
            climbMotorkI = changablekI.getAsDouble();
            climbMotorkD = changablekD.getAsDouble();
            config.closedLoop
                .pid(climbMotorkP, climbMotorkI, climbMotorkD);
            newConfig = true;
        }
        if (changablekV.getAsDouble() != climbMotorkV || changablekG.getAsDouble() != climbMotorkG) {
            climbMotorkV = changablekV.getAsDouble();
            climbMotorkG = changablekG.getAsDouble();
            config.closedLoop.feedForward
                    .kS(0.0)
                    .kV(climbMotorkV)
                    .kA(0.0)
                    .kG(climbMotorkG);
            newConfig = true;
        }
        if (changableCruiseVel.getAsDouble() != climbMotorCruiseVel || changableMaxAccel.getAsDouble() != climbMotorMaxAccel) {
            climbMotorCruiseVel = changableCruiseVel.getAsDouble();
            climbMotorMaxAccel = changableMaxAccel.getAsDouble();
            config.closedLoop.maxMotion
                .cruiseVelocity(climbMotorCruiseVel)
                .maxAcceleration(climbMotorMaxAccel)
                .allowedProfileError(ClimberConstants.CLIMB_ALLOWED_ERROR);
            newConfig = true;
        }
        if (newConfig) {
            climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

}