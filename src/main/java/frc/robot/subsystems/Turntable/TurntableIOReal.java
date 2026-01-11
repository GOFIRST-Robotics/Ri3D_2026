package frc.robot.subsystems.Turntable;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;

public class TurntableIOReal implements TurntableIO {

    private final SparkMax turntableMotorController;
    private final AbsoluteEncoder turntableEncoder;
    private final SparkClosedLoopController turntableClosedLoop;

    public TurntableIOReal()
    {
        turntableMotorController = new SparkMax(Constants.TURRET_HOOD_MOTOR_ID, MotorType.kBrushless);
        turntableEncoder = turntableMotorController.getAbsoluteEncoder();
        turntableClosedLoop = turntableMotorController.getClosedLoopController();
    }

    @Override
    public void updateInputs(TurntableIOInputs inputs) {
        ifOk(turntableMotorController, turntableEncoder::getPosition, (value) -> inputs.turntableRadians = value);
    }

    @Override
    public void setTurntableRadians(double radians) { turntableClosedLoop.setReference(radians * Constants.TURNTABLE_GEAR_RATIO, ControlType.kPosition); }
}