package frc.robot.subsystems.Flywheel;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;

public class FlywheelIOReal implements FlywheelIO {
    private final SparkMax leftFlywheelMotorController;
    private final SparkMax rightFlywheelMotorController;

    private final AbsoluteEncoder leftFlyWheelEncoder;
    private final AbsoluteEncoder rightFlyWheelEncoder;

    private final SparkClosedLoopController leftFlywheelClosedLoop;
    private final SparkClosedLoopController rightFlywheelClosedLoop;

    public FlywheelIOReal()
    {
        leftFlywheelMotorController = new SparkMax(Constants.TURRET_LEFT_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        rightFlywheelMotorController = new SparkMax(Constants.TURRET_RIGHT_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);

        leftFlyWheelEncoder = leftFlywheelMotorController.getAbsoluteEncoder();
        rightFlyWheelEncoder = rightFlywheelMotorController.getAbsoluteEncoder();

        leftFlywheelClosedLoop = leftFlywheelMotorController.getClosedLoopController();
        rightFlywheelClosedLoop = rightFlywheelMotorController.getClosedLoopController();
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        ifOk(leftFlywheelMotorController, leftFlyWheelEncoder::getVelocity, (value) -> inputs.leftFlywheelRPM = value);
        ifOk(rightFlywheelMotorController, rightFlyWheelEncoder::getVelocity, (value) -> inputs.rightFlywheelRPM = value);
    }

    @Override
    public void setLeftFlywheelRPM(double rpm) {  leftFlywheelClosedLoop.setReference(rpm, ControlType.kVelocity); }

    @Override
    public void setRightFlywheelRPM(double rpm) { rightFlywheelClosedLoop.setReference(rpm, ControlType.kVelocity); }
}