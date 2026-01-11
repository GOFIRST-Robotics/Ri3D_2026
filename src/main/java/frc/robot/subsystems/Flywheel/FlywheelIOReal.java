package frc.robot.subsystems.Flywheel;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.util.SparkUtil.ifOk;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;

public class FlywheelIOReal implements FlywheelIO {
    private final SparkMax topFlywheelMotorController;
    private final AbsoluteEncoder topFlyWheelEncoder;
    private final SparkClosedLoopController topFlywheelClosedLoop;
    private LoggedNetworkNumber kPFlywheelTop = new LoggedNetworkNumber("Tuning/FlywheelTop/kP", 0.0);
    private LoggedNetworkNumber kIFlywheelTop = new LoggedNetworkNumber("Tuning/FlywheelTop/kI", 0.0);
    private LoggedNetworkNumber kDFlywheelTop = new LoggedNetworkNumber("Tuning/FlywheelTop/kD", 0.0);
    private double lastkPFlywheelTop;
    private double lastkIFlywheelTop;
    private double lastkDFlywheelTop;

    private final SparkMax bottomFlywheelMotorController;
    private final AbsoluteEncoder bottomFlyWheelEncoder;
    private final SparkClosedLoopController bottomFlywheelClosedLoop;
    private LoggedNetworkNumber kPFlywheelBottom = new LoggedNetworkNumber("Tuning/FlywheelBottom/kP", 0.0);
    private LoggedNetworkNumber kIFlywheelBottom = new LoggedNetworkNumber("Tuning/FlywheelBottom/kI", 0.0);
    private LoggedNetworkNumber kDFlywheelBottom = new LoggedNetworkNumber("Tuning/FlywheelBottom/kD", 0.0);
    private double lastkPFlywheelBottom;
    private double lastkIFlywheelBottom;
    private double lastkDFlywheelBottom;

    private void setInitialMotorPIDs() {
        SparkMaxConfig topFlywheelConfig = new SparkMaxConfig();
        topFlywheelConfig.closedLoop
            .pid(0.0, 0.0, 0.0);
        topFlywheelConfig.closedLoop.feedForward
                .kS(0.0)
                .kV(0.0)
                .kA(0.0)
                .kG(0.0);
        topFlywheelConfig.closedLoop.maxMotion
            .cruiseVelocity(2000)
            .maxAcceleration(10000)
            .allowedProfileError(0.1);
        topFlywheelMotorController.configure(topFlywheelConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        SparkMaxConfig bottomFlywheelConfig = new SparkMaxConfig();
        bottomFlywheelConfig.closedLoop
            .pid(0.0, 0.0, 0.0);
        bottomFlywheelConfig.closedLoop.feedForward
                .kS(0.0)
                .kV(0.0)
                .kA(0.0)
                .kG(0.0);
        bottomFlywheelConfig.closedLoop.maxMotion
            .cruiseVelocity(2000)
            .maxAcceleration(10000)
            .allowedProfileError(0.1);
        bottomFlywheelMotorController.configure(bottomFlywheelConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    public FlywheelIOReal()
    {
        topFlywheelMotorController = new SparkMax(Constants.TURRET_LEFT_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        bottomFlywheelMotorController = new SparkMax(Constants.TURRET_RIGHT_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);

        topFlyWheelEncoder = topFlywheelMotorController.getAbsoluteEncoder();
        bottomFlyWheelEncoder = bottomFlywheelMotorController.getAbsoluteEncoder();

        topFlywheelClosedLoop = topFlywheelMotorController.getClosedLoopController();
        bottomFlywheelClosedLoop = bottomFlywheelMotorController.getClosedLoopController();
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        ifOk(topFlywheelMotorController, topFlyWheelEncoder::getVelocity, (value) -> inputs.topFlywheelRPM = value);
        ifOk(bottomFlywheelMotorController, bottomFlyWheelEncoder::getVelocity, (value) -> inputs.bottomFlywheelRPM = value);
    }

    @Override
    public void setTopFlywheelRPM(double rpm) {  topFlywheelClosedLoop.setSetpoint(rpm, ControlType.kVelocity); }

    @Override
    public void setBottomFlywheelRPM(double rpm) { bottomFlywheelClosedLoop.setSetpoint(rpm, ControlType.kVelocity); }

    public void tunePID() {
        if (kPFlywheelTop.getAsDouble() != lastkPFlywheelTop || kIFlywheelTop.getAsDouble() != lastkIFlywheelTop || kDFlywheelTop.getAsDouble() != lastkDFlywheelTop) {
                SparkMaxConfig config = new SparkMaxConfig();
            config.closedLoop
                .pid(kPFlywheelTop.getAsDouble(), kIFlywheelTop.getAsDouble(), kDFlywheelTop.getAsDouble());
            config.closedLoop.feedForward
                    .kS(0.0)
                    .kV(0.0)
                    .kA(0.0)
                    .kG(0.0);
            config.closedLoop.maxMotion
                .cruiseVelocity(2000)
                .maxAcceleration(10000)
                .allowedProfileError(0.1);
            topFlywheelMotorController.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        }

        if (kPFlywheelBottom.getAsDouble() != lastkPFlywheelBottom || kIFlywheelBottom.getAsDouble() != lastkIFlywheelBottom || kDFlywheelBottom.getAsDouble() != lastkDFlywheelBottom) {
                SparkMaxConfig config = new SparkMaxConfig();
            config.closedLoop
                .pid(kPFlywheelBottom.getAsDouble(), kIFlywheelBottom.getAsDouble(), kDFlywheelBottom.getAsDouble());
            config.closedLoop.feedForward
                    .kS(0.0)
                    .kV(0.0)
                    .kA(0.0)
                    .kG(0.0);
            config.closedLoop.maxMotion
                .cruiseVelocity(2000)
                .maxAcceleration(10000)
                .allowedProfileError(0.1);
            bottomFlywheelMotorController.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        }
    }
}