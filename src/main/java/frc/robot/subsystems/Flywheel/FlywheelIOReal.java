package frc.robot.subsystems.Flywheel;

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

public class FlywheelIOReal implements FlywheelIO {

    private LoggedNetworkNumber tunableTopSpeed;
    private LoggedNetworkNumber tunableBottomSpeed;

    private final SparkMax topFlywheelMotorController;
    private final RelativeEncoder topFlyWheelEncoder;
    private final SparkClosedLoopController topFlywheelClosedLoop;
    private LoggedNetworkNumber changeablekPFlywheelTop;
    private LoggedNetworkNumber changeablekIFlywheelTop;
    private LoggedNetworkNumber changeablekDFlywheelTop;
    private LoggedNetworkNumber changeablekVFlywheelTop;
    private LoggedNetworkNumber changeableMaxAccelTop;
    private double kPFlywheelTop;
    private double kIFlywheelTop;
    private double kDFlywheelTop;
    private double kVFlywheelTop;
    private double maxAccelTop;

    private final SparkMax bottomFlywheelMotorController;
    private final RelativeEncoder bottomFlyWheelEncoder;
    private final SparkClosedLoopController bottomFlywheelClosedLoop;
    private LoggedNetworkNumber changeablekPFlywheelBottom;
    private LoggedNetworkNumber changeablekIFlywheelBottom;
    private LoggedNetworkNumber changeablekDFlywheelBottom;
    private LoggedNetworkNumber changeablekVFlywheelBottom;
    private LoggedNetworkNumber changeableMaxAccelBottom;
    private double kPFlywheelBottom;
    private double kIFlywheelBottom;
    private double kDFlywheelBottom;
    private double kVFlywheelBottom;
    private double maxAccelBottom;

    private void setInitialMotorPIDs() {

        // Around the lowest
        tunableTopSpeed = new LoggedNetworkNumber("/Tuning/FlywheelTop/TuneSpeed", 2000);
        tunableBottomSpeed = new LoggedNetworkNumber("/Tuning/FlywheelBottom/TuneSpeed", 8000);

        kPFlywheelTop = TurretConstants.TOP_FLYWHEEL_KP;
        kIFlywheelTop = TurretConstants.TOP_FLYWHEEL_KI;
        kDFlywheelTop = TurretConstants.TOP_FLYWHEEL_KD;
        kVFlywheelTop = TurretConstants.TOP_FLYWHEEL_KV;
        maxAccelTop = TurretConstants.TOP_FLYWHEEL_ACCEL;
        maxAccelBottom = TurretConstants.BOTTOM_FLYWHEEL_ACCEL;

        changeablekPFlywheelTop = new LoggedNetworkNumber("/Tuning/FlywheelTop/kP", kPFlywheelTop);
        changeablekIFlywheelTop = new LoggedNetworkNumber("/Tuning/FlywheelTop/kI", kIFlywheelTop);
        changeablekDFlywheelTop = new LoggedNetworkNumber("/Tuning/FlywheelTop/kD", kDFlywheelTop);
        changeablekVFlywheelTop = new LoggedNetworkNumber("/Tuning/FlywheelTop/kV", kVFlywheelTop);
        changeableMaxAccelTop = new LoggedNetworkNumber("/Tuning/FlywheelTop/maxAccel", maxAccelTop);

        SparkMaxConfig topFlywheelConfig = new SparkMaxConfig();
        topFlywheelConfig.closedLoop
            .pid(kPFlywheelTop, kIFlywheelTop, kDFlywheelTop);
        topFlywheelConfig.closedLoop.feedForward
                .kS(0.0)
                .kV(kVFlywheelTop)
                .kA(0.0);
        topFlywheelConfig.closedLoop.maxMotion
                .maxAcceleration(maxAccelTop)
                .allowedProfileError(10);
        topFlywheelMotorController.configure(topFlywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        kPFlywheelBottom = TurretConstants.BOTTOM_FLYWHEEL_KP;
        kIFlywheelBottom = TurretConstants.BOTTOM_FLYWHEEL_KI;
        kDFlywheelBottom = TurretConstants.BOTTOM_FLYWHEEL_KD;
        kVFlywheelBottom = TurretConstants.BOTTOM_FLYWHEEL_KV;

        changeablekPFlywheelBottom = new LoggedNetworkNumber("/Tuning/FlywheelBottom/kP", kPFlywheelBottom);
        changeablekIFlywheelBottom = new LoggedNetworkNumber("/Tuning/FlywheelBottom/kI", kIFlywheelBottom);
        changeablekDFlywheelBottom = new LoggedNetworkNumber("/Tuning/FlywheelBottom/kD", kDFlywheelBottom);
        changeablekVFlywheelBottom = new LoggedNetworkNumber("/Tuning/FlywheelBottom/kV", kVFlywheelBottom);
        changeableMaxAccelBottom = new LoggedNetworkNumber("/Tuning/FlywheelBottom/maxAccel", maxAccelBottom);

        SparkMaxConfig bottomFlywheelConfig = new SparkMaxConfig();
        bottomFlywheelConfig.closedLoop
            .pid(kPFlywheelBottom, kIFlywheelBottom, kDFlywheelBottom);
        bottomFlywheelConfig.closedLoop.feedForward
                .kS(0.0)
                .kV(kVFlywheelBottom)
                .kA(0.0);
        bottomFlywheelConfig.closedLoop.maxMotion
                .maxAcceleration(maxAccelBottom)
                .allowedProfileError(10);
        bottomFlywheelMotorController.configure(bottomFlywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public FlywheelIOReal()
    {
        topFlywheelMotorController = new SparkMax(Constants.TURRET_LEFT_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        bottomFlywheelMotorController = new SparkMax(Constants.TURRET_RIGHT_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);

        topFlyWheelEncoder = topFlywheelMotorController.getEncoder();
        bottomFlyWheelEncoder = bottomFlywheelMotorController.getEncoder();

        topFlywheelClosedLoop = topFlywheelMotorController.getClosedLoopController();
        bottomFlywheelClosedLoop = bottomFlywheelMotorController.getClosedLoopController();

        setInitialMotorPIDs();
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        ifOk(topFlywheelMotorController, topFlyWheelEncoder::getVelocity, (value) -> inputs.topFlywheelRPM = value);
        ifOk(bottomFlywheelMotorController, bottomFlyWheelEncoder::getVelocity, (value) -> inputs.bottomFlywheelRPM = value);
    }

    @Override
    public void setTopFlywheelRPM(double rpm) {  topFlywheelClosedLoop.setSetpoint(rpm, ControlType.kMAXMotionVelocityControl); }

    @Override
    public void setBottomFlywheelRPM(double rpm) { bottomFlywheelClosedLoop.setSetpoint(rpm, ControlType.kMAXMotionVelocityControl); }

    @Override
    public void tuneTopAndBottomRPM() {

        topFlywheelClosedLoop.setSetpoint(tunableTopSpeed.getAsDouble(), ControlType.kMAXMotionVelocityControl);
        bottomFlywheelClosedLoop.setSetpoint(tunableBottomSpeed.getAsDouble(), ControlType.kMAXMotionVelocityControl);

    }

    @Override
    public void periodic() {
        boolean topHasChanged = false;
        SparkMaxConfig topConfig = new SparkMaxConfig();
        if (changeablekPFlywheelTop.getAsDouble() != kPFlywheelTop || changeablekIFlywheelTop.getAsDouble() != kIFlywheelTop || changeablekDFlywheelTop.getAsDouble() != kDFlywheelTop) {
            kPFlywheelTop = changeablekPFlywheelTop.getAsDouble();
            kIFlywheelTop = changeablekIFlywheelTop.getAsDouble();
            kDFlywheelTop = changeablekDFlywheelTop.getAsDouble();
            topConfig.closedLoop
                .pid(kPFlywheelTop, kIFlywheelTop, kDFlywheelTop);
            topHasChanged = true;
        }
        if (changeablekVFlywheelTop.getAsDouble() != kVFlywheelTop) {
            kVFlywheelTop = changeablekVFlywheelTop.getAsDouble();
            topConfig.closedLoop.feedForward
                    .kS(0.0)
                    .kV(kVFlywheelTop)
                    .kA(0.0);
            topHasChanged = true;
        }
        if (changeableMaxAccelTop.getAsDouble() != maxAccelTop) {
            maxAccelTop = changeableMaxAccelTop.getAsDouble();
            topConfig.closedLoop.maxMotion
                    .maxAcceleration(maxAccelTop)
                    .allowedProfileError(10);
            topHasChanged = true;
        }
        if (topHasChanged) {
            topFlywheelMotorController.configure(topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

        boolean bottomHasChanged = false;
        SparkMaxConfig bottomConfig = new SparkMaxConfig();

        if (changeablekPFlywheelBottom.getAsDouble() != kPFlywheelBottom || changeablekIFlywheelBottom.getAsDouble() != kIFlywheelBottom || changeablekDFlywheelBottom.getAsDouble() != kDFlywheelBottom) {
            kPFlywheelBottom = changeablekPFlywheelBottom.getAsDouble();
            kIFlywheelBottom = changeablekIFlywheelBottom.getAsDouble();
            kDFlywheelBottom = changeablekDFlywheelBottom.getAsDouble();
            bottomConfig.closedLoop
                .pid(kPFlywheelBottom, kIFlywheelBottom, kDFlywheelBottom);
            bottomHasChanged = true;
        }
        if (changeablekVFlywheelBottom.getAsDouble() != kVFlywheelBottom) {
            kVFlywheelBottom = changeablekVFlywheelBottom.getAsDouble();
            bottomConfig.closedLoop.feedForward
                    .kS(0.0)
                    .kV(kVFlywheelBottom)
                    .kA(0.0);
            bottomHasChanged = true;
        }
        if (changeableMaxAccelBottom.getAsDouble() != maxAccelBottom) {
            maxAccelBottom = changeableMaxAccelBottom.getAsDouble();
            bottomConfig.closedLoop.maxMotion
                    .maxAcceleration(maxAccelBottom)
                    .allowedProfileError(10);
            bottomHasChanged = true;
        }
        if (bottomHasChanged) {
            bottomFlywheelMotorController.configure(bottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }
}