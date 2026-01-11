package frc.robot.subsystems.Climber;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
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


    public ClimberIOReal() {
        climbMotor = new SparkMax(ClimberConstants.CLIMBER_MOTOR_CAN_ID, MotorType.kBrushless);
        climbPID = climbMotor.getClosedLoopController();
        setInitialMotorPIDs();
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
                climbPID.setReference(ClimberConstants.CLIMBER_ZERO_POS, ControlType.kMAXMotionPositionControl);
                break;
            case AUTO:
                climbPID.setReference(ClimberConstants.AUTO_CLIMB_POS, ControlType.kMAXMotionPositionControl);
                break;
            case RUNG_ONE:
                climbPID.setReference(ClimberConstants.RUNG_ONE_CLIMBER_POS, ControlType.kMAXMotionPositionControl);
                break;
            case RUNG_TWO:
                climbPID.setReference(ClimberConstants.RUNG_TWO_CLIMBER_POS, ControlType.kMAXMotionPositionControl);
                break;
            case RUNG_THREE:
                climbPID.setReference(ClimberConstants.RUNG_THREE_CLIMBER_POS, ControlType.kMAXMotionPositionControl);
                break;
        }
    }

    @Override
    public void resetClimber() {
        climbMotor.setVoltage(-5);
    }

    public void tunePID() {
        if (kP.getAsDouble() != lastkP) {
            
        }
    }


}