package frc.robot.subsystems.Hood;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
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
    private final AbsoluteEncoder hoodEncoder;
    private final SparkClosedLoopController hoodClosedLoop;

    private LoggedNetworkNumber changeablekP;
    private LoggedNetworkNumber changeablekI;
    private LoggedNetworkNumber changeablekD;
    private LoggedNetworkNumber changeablekV;

    private double kP;
    private double kI;
    private double kD;
    private double kV;

    private void setInitialMotorPIDs() {
        kP = TurretConstants.HOOD_kP;
        kI = TurretConstants.HOOD_kI;
        kD = TurretConstants.HOOD_kD;
        kV = TurretConstants.HOOD_kV;

        changeablekP = new LoggedNetworkNumber("Tuning/Hood/kP", kP);
        changeablekI = new LoggedNetworkNumber("Tuning/Hood/kP", kI);
        changeablekD = new LoggedNetworkNumber("Tuning/Hood/kP", kD);
        changeablekV = new LoggedNetworkNumber("Tuning/Hood/kP", kV);

        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
            .pid(kP, kI, kD);
        config.closedLoop.feedForward
                .kS(0.0)
                .kV(kV)
                .kA(0.0)
                .kG(0.0);
        config.softLimit
            .forwardSoftLimit(TurretConstants.TURRET_HOOD_MOTOR_MAX_ROTATIONS)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(TurretConstants.TURRET_HOOD_MOTOR_MIN_ROTATIONS)
            .reverseSoftLimitEnabled(true);
        hoodMotorController.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
        periodic();

        ifOk(hoodMotorController, hoodEncoder::getPosition, (value) -> inputs.hoodRadians = value);
    }

    @Override
    public void setHoodRadians(double radians)
    { 
        double clampedRadians = radians;
        if (clampedRadians < TurretConstants.TURRET_HOOD_MIN_RADIANS) { clampedRadians = TurretConstants.TURRET_HOOD_MIN_RADIANS; }
        else if (clampedRadians > TurretConstants.TURRET_HOOD_MAX_RADIANS) {clampedRadians = TurretConstants.TURRET_HOOD_MAX_RADIANS; }

        hoodClosedLoop.setSetpoint(radians * TurretConstants.TURRET_HOOD_GEAR_RATIO, ControlType.kPosition);
    }

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
        if (changeablekV.getAsDouble() != kV) {
            kV = changeablekV.getAsDouble();
            config.closedLoop.feedForward
                    .kS(0.0)
                    .kV(0.0)
                    .kA(kV)
                    .kG(0.0);
            hasChanged = true;
        }
        if (hasChanged) {
            hoodMotorController.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }
}