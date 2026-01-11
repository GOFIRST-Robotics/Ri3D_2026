package frc.robot.subsystems.Hood;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.util.SparkUtil.ifOk;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;

public class HoodIOReal implements HoodIO {

    private final SparkMax hoodMotorController;
    private final AbsoluteEncoder hoodEncoder;
    private final SparkClosedLoopController hoodClosedLoop;

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
        config.softLimit
            .forwardSoftLimit(Constants.TURRET_HOOD_MOTOR_MAX_ROTATIONS)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(Constants.TURRET_HOOD_MOTOR_MIN_ROTATIONS)
            .reverseSoftLimitEnabled(true);
        hoodMotorController.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    public HoodIOReal()
    {
        hoodMotorController = new SparkMax(Constants.TURRET_HOOD_MOTOR_ID, MotorType.kBrushless);
        hoodEncoder = hoodMotorController.getAbsoluteEncoder();
        hoodClosedLoop = hoodMotorController.getClosedLoopController();

        setInitialMotorPIDs();
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        tunePID();

        ifOk(hoodMotorController, hoodEncoder::getPosition, (value) -> inputs.hoodRadians = value);
    }

    @Override
    public void setHoodRadians(double radians)
    { 
        double clampedRadians = radians;
        if (clampedRadians < Constants.TURRET_HOOD_MIN_RADIANS) { clampedRadians = Constants.TURRET_HOOD_MIN_RADIANS; }
        else if (clampedRadians > Constants.TURRET_HOOD_MAX_RADIANS) {clampedRadians = Constants.TURRET_HOOD_MAX_RADIANS; }

        hoodClosedLoop.setSetpoint(radians * Constants.TURRET_HOOD_GEAR_RATIO, ControlType.kPosition);
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
            hoodMotorController.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        }
    }
}