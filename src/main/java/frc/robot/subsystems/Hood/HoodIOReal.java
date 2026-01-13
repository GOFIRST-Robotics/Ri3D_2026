package frc.robot.subsystems.Hood;

import com.fasterxml.jackson.databind.cfg.ConfigOverride;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.util.SparkUtil.ifOk;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

public class HoodIOReal implements HoodIO {

    private final SparkMax hoodMotorController;
    private final RelativeEncoder hoodEncoder;
    private final SparkClosedLoopController hoodClosedLoop;

    private LoggedNetworkNumber changeablekP;
    private LoggedNetworkNumber changeablekI;
    private LoggedNetworkNumber changeablekD;
    private LoggedNetworkNumber changeablekV;
    private LoggedNetworkNumber changeablekCos;

    private double kP;
    private double kI;
    private double kD;
    private double kV;
    private double kCos;

    private void setInitialMotorPIDs() {
        kP = TurretConstants.HOOD_kP;
        kI = TurretConstants.HOOD_kI;
        kD = TurretConstants.HOOD_kD;
        kV = TurretConstants.HOOD_kV;
        kCos = TurretConstants.HOOD_kCos;

        changeablekP = new LoggedNetworkNumber("/Tuning/Hood/kP", kP);
        changeablekI = new LoggedNetworkNumber("/Tuning/Hood/kI", kI);
        changeablekD = new LoggedNetworkNumber("/Tuning/Hood/kD", kD);
        changeablekV = new LoggedNetworkNumber("/Tuning/Hood/kV", kV);
        changeablekCos = new LoggedNetworkNumber("/Tuning/Hood/kCos", kCos);

        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
            .pid(kP, kI, kD);
        config.closedLoop.feedForward
                .kS(0.0)
                .kV(kV)
                .kA(0.0)
                .kCos(kCos)
                .kCosRatio(Constants.TurretConstants.TURRET_HOOD_GEAR_RATIO / Constants.TWO_PI);
        config.softLimit
            .forwardSoftLimit(TurretConstants.TURRET_HOOD_MOTOR_MAX_ROTATIONS)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(TurretConstants.TURRET_HOOD_MOTOR_MIN_ROTATIONS)
            .reverseSoftLimitEnabled(true);
        config.closedLoop.maxMotion
            .cruiseVelocity(Constants.TurretConstants.HOOD_CRUISE_VEL)
            .maxAcceleration(Constants.TurretConstants.HOOD_MAX_ACCEL)
            //.cruiseVelocity(10000)
            .allowedProfileError(Constants.TurretConstants.HOOD_ALLOWED_PROFILE_ERROR);
        config.inverted(true);
        hoodMotorController.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public HoodIOReal()
    {
        hoodMotorController = new SparkMax(Constants.TURRET_HOOD_MOTOR_ID, MotorType.kBrushless);
        hoodEncoder = hoodMotorController.getEncoder();
        hoodClosedLoop = hoodMotorController.getClosedLoopController();

        setInitialMotorPIDs();
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        periodic();

        ifOk(hoodMotorController, hoodEncoder::getPosition, (value) -> inputs.motorPosition = value);
        inputs.hoodRadians = (inputs.motorPosition / TurretConstants.TURRET_HOOD_GEAR_RATIO) * Constants.TWO_PI;
        inputs.targetPosition = hoodClosedLoop.getSetpoint();
    }

    @Override
    public void setHoodRadians(double radians)
    {
        double clampedRadians = radians;
        if (clampedRadians < 0) { clampedRadians = 0; }
        else if (clampedRadians > TurretConstants.TURRET_HOOD_RANGE_RADIANS) { clampedRadians = TurretConstants.TURRET_HOOD_RANGE_RADIANS; }

        double motorRotations = (clampedRadians / Constants.TWO_PI) * TurretConstants.TURRET_HOOD_GEAR_RATIO;
        hoodClosedLoop.setSetpoint(motorRotations, ControlType.kMAXMotionPositionControl);
        System.out.println(motorRotations);
    }

    @Override
    public void periodic() {
        boolean hasChanged = false;
        SparkMaxConfig config = new SparkMaxConfig();
        if (changeablekP.getAsDouble() != kP || changeablekI.getAsDouble() != kI || changeablekD.getAsDouble() != kD) {
            kP = changeablekP.getAsDouble();
            kI = changeablekI.getAsDouble();
            kD = changeablekD.getAsDouble();
            config.closedLoop
                .pid(kP, kI, kD);
            hasChanged = true;
        }
        if (changeablekV.getAsDouble() != kV || changeablekCos.getAsDouble() != kCos) {
            kV = changeablekV.getAsDouble();
            kCos = changeablekCos.getAsDouble();
            config.closedLoop.feedForward
                    .kS(0.0)
                    .kV(kV)
                    .kA(0.0)
                    .kCos(kCos)
                    .kCosRatio(1 / Constants.TurretConstants.TURRET_HOOD_GEAR_RATIO);
            hasChanged = true;
        }
        if (hasChanged) {
            config.inverted(true);
            hoodMotorController.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }
}