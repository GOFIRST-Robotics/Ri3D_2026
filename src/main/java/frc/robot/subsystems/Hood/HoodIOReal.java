package frc.robot.subsystems.Hood;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;

public class HoodIOReal implements HoodIO {

    private final SparkMax hoodMotorController;
    private final AbsoluteEncoder hoodEncoder;
    private final SparkClosedLoopController hoodClosedLoop;

    public HoodIOReal()
    {
        hoodMotorController = new SparkMax(Constants.TURRET_HOOD_MOTOR_ID, MotorType.kBrushless);
        hoodEncoder = hoodMotorController.getAbsoluteEncoder();
        hoodClosedLoop = hoodMotorController.getClosedLoopController();
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        ifOk(hoodMotorController, hoodEncoder::getPosition, (value) -> inputs.hoodRadians = value);
    }

    @Override
    public void setHoodRadians(double radians) { hoodClosedLoop.setReference(radians * Constants.HOOD_GEAR_RATIO, ControlType.kPosition); }
}