package frc.robot.subsystems.drive.MecanumDrive;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants.MecanumConstants;

import java.util.function.DoubleSupplier;

public class MecanumModuleIOSpark implements MecanumModuleIO {
  private final SparkBase spark;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;

  private final Debouncer connectedDebounce = new Debouncer(0.5);

  /**
   * @param wheelIndex 0=FL, 1=FR, 2=BL, 3=BR (or any mapping you choose)
   */
  public MecanumModuleIOSpark(int wheelIndex) {
    int canId =
        switch (wheelIndex) {
          case 0 -> MecanumConstants.frontLeftCanId;
          case 1 -> MecanumConstants.frontRightCanId;
          case 2 -> MecanumConstants.backLeftCanId;
          case 3 -> MecanumConstants.backRightCanId;
          default -> 0;
        };

    boolean inverted =
        switch (wheelIndex) {
          case 0 -> MecanumConstants.frontLeftInverted;
          case 1 -> MecanumConstants.frontRightInverted;
          case 2 -> MecanumConstants.backLeftInverted;
          case 3 -> MecanumConstants.backRightInverted;
          default -> false;
        };

    spark = new SparkMax(canId, MotorType.kBrushless);
    encoder = spark.getEncoder();
    controller = spark.getClosedLoopController();

    var config = new SparkMaxConfig();
    config
        .inverted(inverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(MecanumConstants.driveMotorCurrentLimit)
        .voltageCompensation(12.0);

    // Convert encoder units to radians / rad/s so the rest of the codebase matches swerve style
    config
        .encoder
        .positionConversionFactor(MecanumConstants.encoderPositionFactorRad)
        .velocityConversionFactor(MecanumConstants.encoderVelocityFactorRadPerSec);

    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(MecanumConstants.kP, 0.0, MecanumConstants.kD, 0.0);

    config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((20))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        spark,
        5,
        () -> spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Optional: zero encoder at boot (or handle zeroing elsewhere)
    tryUntilOk(spark, 5, () -> encoder.setPosition(0.0));

  }

  @Override
  public void updateInputs(MecanumModuleIOInputs inputs) {
    sparkStickyFault = false;

    ifOk(spark, encoder::getPosition, (value) -> inputs.wheelPositionRad = value);
    ifOk(spark, encoder::getVelocity, (value) -> inputs.wheelVelocityRadPerSec = value);
    ifOk(
        spark,
        new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(spark, spark::getOutputCurrent, (value) -> inputs.currentAmps = value);

    inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setOpenLoopVolts(double volts) {
    spark.setVoltage(volts);
  }

  @Override
  public void setWheelVelocity(double velocityRadPerSec) {
    double ffVolts = MecanumConstants.kS * Math.signum(velocityRadPerSec) + MecanumConstants.kV * velocityRadPerSec;
    controller.setReference(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }
}