package frc.robot.subsystems.Climber;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.ClimberConstants;

public class ClimberIOReal implements ClimberIO {
    private Servo servo1;
    private Servo servo2;
    private SparkMax climbMotor;
    private SparkClosedLoopController climbPID;

    private LoggedNetworkNumber changablekPSlot0;
    private LoggedNetworkNumber changablekISlot0;
    private LoggedNetworkNumber changablekDSlot0;
    private LoggedNetworkNumber changablekVSlot0;
    private LoggedNetworkNumber changablekGSlot0;

    private LoggedNetworkNumber changablekPSlot1;
    private LoggedNetworkNumber changablekISlot1;
    private LoggedNetworkNumber changablekDSlot1;
    private LoggedNetworkNumber changablekVSlot1;
    private LoggedNetworkNumber changablekGSlot1;

    private LoggedNetworkNumber changableCruiseVel;
    private LoggedNetworkNumber changableMaxAccel;

    private double climbMotorkPSlot0;
    private double climbMotorkISlot0;
    private double climbMotorkDSlot0;
    private double climbMotorkVSlot0;
    private double climbMotorkGSlot0;

    private double climbMotorkPSlot1;
    private double climbMotorkISlot1;
    private double climbMotorkDSlot1;
    private double climbMotorkVSlot1;
    private double climbMotorkGSlot1;

    private double climbMotorCruiseVel;
    private double climbMotorMaxAccel;

    public ClimberIOReal() {
        climbMotor = new SparkMax(ClimberConstants.CLIMBER_MOTOR_CAN_ID, MotorType.kBrushless);
        climbPID = climbMotor.getClosedLoopController();
        servo1 = new Servo(0);
        servo2 = new Servo(1);
        setInitialMotorSettings();
    }

    private void setInitialMotorSettings() {
        climbMotorkPSlot0 = ClimberConstants.CLIMB_MOTOR_KP_SLOT0;
        climbMotorkISlot0 = ClimberConstants.CLIMB_MOTOR_KI_SLOT0;
        climbMotorkDSlot0 = ClimberConstants.CLIMB_MOTOR_KD_SLOT0;
        climbMotorkVSlot0 = ClimberConstants.CLIMB_MOTOR_KV_SLOT0;
        climbMotorkGSlot0 = ClimberConstants.CLIMB_MOTOR_KG_SLOT0;

        climbMotorkPSlot1 = ClimberConstants.CLIMB_MOTOR_KP_SLOT1;
        climbMotorkISlot1 = ClimberConstants.CLIMB_MOTOR_KI_SLOT1;
        climbMotorkDSlot1 = ClimberConstants.CLIMB_MOTOR_KD_SLOT1;
        climbMotorkVSlot1 = ClimberConstants.CLIMB_MOTOR_KV_SLOT1;
        climbMotorkGSlot1 = ClimberConstants.CLIMB_MOTOR_KG_SLOT1;

        climbMotorCruiseVel = ClimberConstants.CLIMB_MOTOR_CRUISE_VEL;
        climbMotorMaxAccel = ClimberConstants.CLIMB_MOTOR_MAX_ACCEL;

        changablekPSlot0 = new LoggedNetworkNumber("Tuning/Climber/kP", climbMotorkPSlot0);
        changablekISlot0 = new LoggedNetworkNumber("Tuning/Climber/kI", climbMotorkISlot0);
        changablekDSlot0 = new LoggedNetworkNumber("Tuning/Climber/kD", climbMotorkDSlot0);
        changablekVSlot0 = new LoggedNetworkNumber("Tuning/Climber/kV", climbMotorkVSlot0);
        changablekGSlot0 = new LoggedNetworkNumber("Tuning/Climber/kG", climbMotorkGSlot0);
        changableCruiseVel = new LoggedNetworkNumber("Tuning/Climber/CruiseVel", climbMotorCruiseVel);
        changableMaxAccel = new LoggedNetworkNumber("Tuning/Climber/MaxAccel", climbMotorMaxAccel);

        climbMotor.getEncoder().setPosition(0);

        SparkMaxConfig configSlot0 = new SparkMaxConfig();
        configSlot0.closedLoop
            .pid(climbMotorkPSlot0, climbMotorkISlot0, climbMotorkDSlot0, ClosedLoopSlot.kSlot0);
        configSlot0.closedLoop.feedForward
                .kS(0.0, ClosedLoopSlot.kSlot0)
                .kV(climbMotorkVSlot0, ClosedLoopSlot.kSlot0)
                .kA(0.0, ClosedLoopSlot.kSlot0)
                .kG(climbMotorkGSlot0, ClosedLoopSlot.kSlot0);
        configSlot0.closedLoop.maxMotion
            .cruiseVelocity(climbMotorCruiseVel, ClosedLoopSlot.kSlot0)
            .maxAcceleration(climbMotorMaxAccel, ClosedLoopSlot.kSlot0)
            .allowedProfileError(ClimberConstants.CLIMB_ALLOWED_ERROR);

        SparkMaxConfig configSlot1 = new SparkMaxConfig();
        configSlot1.closedLoop
            .pid(climbMotorkPSlot1, climbMotorkISlot1, climbMotorkDSlot1, ClosedLoopSlot.kSlot1);
        configSlot1.closedLoop.feedForward
                .kS(0.0, ClosedLoopSlot.kSlot1)
                .kV(climbMotorkVSlot0, ClosedLoopSlot.kSlot1)
                .kA(0.0, ClosedLoopSlot.kSlot1)
                .kG(climbMotorkGSlot0, ClosedLoopSlot.kSlot1);
        configSlot0.closedLoop.maxMotion
            .cruiseVelocity(climbMotorCruiseVel, ClosedLoopSlot.kSlot1)
            .maxAcceleration(climbMotorMaxAccel, ClosedLoopSlot.kSlot1)
            .allowedProfileError(ClimberConstants.CLIMB_ALLOWED_ERROR, ClosedLoopSlot.kSlot1);
        
        SparkMaxConfig limitConfig = new SparkMaxConfig();
        configSlot0.softLimit
            .forwardSoftLimit(ClimberConstants.CLIMB_MAX_POSITION_ALLOWED)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(ClimberConstants.CLIMB_MIN_POSITION_ALLOWED)
            .reverseSoftLimitEnabled(true);
        configSlot0.inverted(false);
        limitConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        climbMotor.configure(configSlot0, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climbMotor.configure(configSlot1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climbMotor.configure(limitConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ClimberIOInputsAutoLogged inputs) {
        inputs.climberPosInRotations = climbMotor.getEncoder().getPosition();
    }

    @Override
    public void setClimbPositionWithLoad(ClimbPosition position) {
        switch (position) {
            case ZERO:
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
    public void setServoOnePos(double position) {
        servo1.setPosition(position);
    }

    @Override
    public void setServoTwoPos(double position) {
        servo2.setPosition(position);
    }

    @Override
    public boolean isMovementFinished() {
        return Math.abs(climbPID.getSetpoint() - climbMotor.getEncoder().getPosition()) < ClimberConstants.CLIMB_ALLOWED_ERROR;
    }

    @Override
    public void periodic() {
        boolean newConfig = false;
        SparkMaxConfig config = new SparkMaxConfig();
        if (changablekPSlot0.getAsDouble() != climbMotorkPSlot0 || changablekISlot0.getAsDouble() != climbMotorkISlot0 || changablekDSlot0.getAsDouble() != climbMotorkDSlot0) {
            climbMotorkPSlot0 = changablekPSlot0.getAsDouble();
            climbMotorkISlot0 = changablekISlot0.getAsDouble();
            climbMotorkDSlot0 = changablekDSlot0.getAsDouble();
            config.closedLoop
                .pid(climbMotorkPSlot0, climbMotorkISlot0, climbMotorkDSlot0);
            newConfig = true;
        }
        if (changablekVSlot0.getAsDouble() != climbMotorkVSlot0 || changablekGSlot0.getAsDouble() != climbMotorkGSlot0) {
            climbMotorkVSlot0 = changablekVSlot0.getAsDouble();
            climbMotorkGSlot0 = changablekGSlot0.getAsDouble();
            config.closedLoop.feedForward
                    .kS(0.0)
                    .kV(climbMotorkVSlot0)
                    .kA(0.0)
                    .kG(climbMotorkGSlot0);
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