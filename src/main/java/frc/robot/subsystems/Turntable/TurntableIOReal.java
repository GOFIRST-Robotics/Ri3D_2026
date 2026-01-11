package frc.robot.subsystems.Turntable;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.util.SparkUtil.ifOk;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;

public class TurntableIOReal implements TurntableIO {

    private final SparkMax turntableMotorController;
    private final AbsoluteEncoder turntableEncoder;
    private final SparkClosedLoopController turntableClosedLoop;

    private LoggedNetworkNumber kP = new LoggedNetworkNumber("Tuning/Hood/kP", 0.0);
    private LoggedNetworkNumber kI = new LoggedNetworkNumber("Tuning/Hood/kI", 0.0);
    private LoggedNetworkNumber kD = new LoggedNetworkNumber("Tuning/Hood/kD", 0.0);

    private double lastkP;
    private double lastkI;
    private double lastkD;

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
        turntableMotorController.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    public TurntableIOReal()
    {
        turntableMotorController = new SparkMax(Constants.TURRET_HOOD_MOTOR_ID, MotorType.kBrushless);
        turntableEncoder = turntableMotorController.getAbsoluteEncoder();
        turntableClosedLoop = turntableMotorController.getClosedLoopController();

        setInitialMotorPIDs();
    }

    @Override
    public void updateInputs(TurntableIOInputs inputs) {
        tunePID();

        ifOk(turntableMotorController, turntableEncoder::getPosition, (value) -> inputs.turntableRadians = value);
    }

    @Override
    public void setTurntableRadians(double radians)
    { 
        double clampedRadians = radians;
        if (clampedRadians < -Constants.TURRET_TURNTABLE_MAX_RADIANS) { clampedRadians = -Constants.TURRET_TURNTABLE_MAX_RADIANS; }
        else if (clampedRadians > Constants.TURRET_TURNTABLE_MAX_RADIANS) {clampedRadians = Constants.TURRET_TURNTABLE_MAX_RADIANS; }

        turntableClosedLoop.setSetpoint(radians * Constants.TURRET_TURNTABLE_GEAR_RATIO, ControlType.kPosition); 
    }

        public void tunePID() {
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
            turntableMotorController.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        }
    }
}