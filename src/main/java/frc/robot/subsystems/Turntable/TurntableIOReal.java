package frc.robot.subsystems.Turntable;

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

public class TurntableIOReal implements TurntableIO {

    private final SparkMax turntableMotorController;
    private final RelativeEncoder turntableEncoder;
    private final SparkClosedLoopController turntableClosedLoop;

    private LoggedNetworkNumber changeablekP;
    private LoggedNetworkNumber changeablekI;
    private LoggedNetworkNumber changeablekD;
    private LoggedNetworkNumber changeablekS;
    private LoggedNetworkNumber changeablekV;
    private LoggedNetworkNumber changeablekA;
    private LoggedNetworkNumber changeableCruiseVel;
    private LoggedNetworkNumber changeableMaxAccel;

    private double kP;
    private double kI;
    private double kD;
    private double kS;
    private double kV;
    private double kA;
    private double cruiseVel;
    private double maxAccel;

    private void setInitialMotorPIDs() {
        kP = TurretConstants.TURNTABLE_kP;
        kI = TurretConstants.TURNTABLE_kI;
        kD = TurretConstants.TURNTABLE_kD;
        kS = TurretConstants.TURNTABLE_kS;
        kV = TurretConstants.TURNTABLE_kV;
        kA = TurretConstants.TURNTABLE_kA;
        cruiseVel = TurretConstants.TURNTABLE_CRUISE_VEL;
        maxAccel = TurretConstants.TURNTABLE_MAX_ACCEL;
        changeablekP = new LoggedNetworkNumber("Tuning/Turntable/kP", kP);
        changeablekI = new LoggedNetworkNumber("Tuning/Turntable/kI", kI);
        changeablekD = new LoggedNetworkNumber("Tuning/Turntable/kD", kD);
        changeablekS = new LoggedNetworkNumber("Tuning/Turntable/kS", kS);
        changeablekV = new LoggedNetworkNumber("Tuning/Turntable/kV", kV);
        changeablekA = new LoggedNetworkNumber("Tuning/Turntable/kA", kA);
        changeableCruiseVel = new LoggedNetworkNumber("Tuning/Turntable/cruiseVel", cruiseVel);
        changeableMaxAccel = new LoggedNetworkNumber("Tuning/Turntable/maxAccel", maxAccel);

        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
            .pid(kP, kI, kD);
        config.closedLoop.feedForward
                .kS(kS)
                .kV(kV)
                .kA(kA)
                .kG(0.0);
        config.closedLoop.maxMotion
            .cruiseVelocity(cruiseVel)
            .maxAcceleration(maxAccel)
            .allowedProfileError(TurretConstants.TURNTABLE_ALLOWED_ERROR);
        turntableMotorController.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public TurntableIOReal()
    {
        turntableMotorController = new SparkMax(Constants.TURRET_HOOD_MOTOR_ID, MotorType.kBrushless);
        turntableEncoder = turntableMotorController.getEncoder();
        turntableClosedLoop = turntableMotorController.getClosedLoopController();

        setInitialMotorPIDs();
    }

    @Override
    public void updateInputs(TurntableIOInputs inputs) {
        periodic();

        ifOk(turntableMotorController, turntableEncoder::getPosition, (value) -> inputs.turntableRadians = (value / TurretConstants.TURRET_TURNTABLE_GEAR_RATIO) * Constants.TWO_PI);
    }

    @Override
    public void setTurntableRadians(double radians)
    { 
        double clampedRadians = radians;
        if (clampedRadians < -TurretConstants.TURRET_TURNTABLE_MAX_RADIANS) { clampedRadians = -TurretConstants.TURRET_TURNTABLE_MAX_RADIANS; }
        else if (clampedRadians > TurretConstants.TURRET_TURNTABLE_MAX_RADIANS) {clampedRadians = TurretConstants.TURRET_TURNTABLE_MAX_RADIANS; }

        double motorRotations = (clampedRadians / Constants.TWO_PI) * TurretConstants.TURRET_TURNTABLE_GEAR_RATIO;
        turntableClosedLoop.setSetpoint(motorRotations, ControlType.kMAXMotionPositionControl); 
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

        if (changeablekS.getAsDouble() != kS || changeablekV.getAsDouble() != kV || changeablekA.getAsDouble() != kA) {
            kS = changeablekS.getAsDouble();
            kV = changeablekV.getAsDouble();
            kA = changeablekA.getAsDouble();
            config.closedLoop.feedForward
                    .kS(kS)
                    .kV(kV)
                    .kA(kA)
                    .kG(0.0);
            hasChanged = true;
        }

        if (changeableCruiseVel.getAsDouble() != cruiseVel || changeableMaxAccel.getAsDouble() != maxAccel) {
            cruiseVel = changeableCruiseVel.getAsDouble();
            maxAccel = changeableMaxAccel.getAsDouble();
            config.closedLoop.maxMotion
                .cruiseVelocity(cruiseVel)
                .maxAcceleration(maxAccel)
                .allowedProfileError(TurretConstants.TURNTABLE_ALLOWED_ERROR);
            hasChanged = true;
        }

        if (hasChanged) {
            turntableMotorController.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }
}