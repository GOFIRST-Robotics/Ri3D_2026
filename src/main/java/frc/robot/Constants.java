// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

public class MecanumConstants {
  private MecanumConstants() {}

  // CAN IDs
  public static final int frontLeftCanId = 1;
  public static final int frontRightCanId = 2;
  public static final int backLeftCanId = 3;
  public static final int backRightCanId = 4;

  // Inversions (set to match your wiring/gearboxes)
  public static final boolean frontLeftInverted = false;
  public static final boolean frontRightInverted = true;
  public static final boolean backLeftInverted = false;
  public static final boolean backRightInverted = true;

  public static final int driveMotorCurrentLimit = 60;

  // Encoder conversion factors
  // If using NEO internal encoder: 1 motor revolution = 2π rad
  // If you want wheel radians, multiply by (2π / gearRatio). Decide and be consistent.
  public static final double encoderPositionFactorRad = 2.0 * Math.PI; // rad per motor rev
  public static final double encoderVelocityFactorRadPerSec = (2.0 * Math.PI) / 60.0; // rad/s per RPM

  // Closed-loop gains (in rad/s units)
  public static final double kP = 0.0;
  public static final double kD = 0.0;

  // Feedforward in volts (V = kS*sign(ω) + kV*ω)
  public static final double kS = 0.0;
  public static final double kV = 0.0;
}

public static final double WHEEL_DIAMETER = Units.inchesToMeters(6); // Convert from inches to meters
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; // Measured in meters
	public static final double TRACK_WIDTH = Units.inchesToMeters(21); // Distance between centers of right and left wheels on robot (in meters)
    public static final double WHEEL_BASE = Units.inchesToMeters(16.75); // Distance between centers of front and back wheels on robot (in meters)

    // Controller Input Axes //
    public static final int CONTROLLER_USB_PORT_ID = 0; // USB port that the controller is plugged in to
    public static final int RIGHT_VERTICAL_JOYSTICK_AXIS = 3;
    public static final int RIGHT_HORIZONTAL_JOYSTICK_AXIS = 2;
    public static final int LEFT_VERTICAL_JOYSTICK_AXIS = 1;
    public static final int LEFT_HORIZONTAL_JOYSTICK_AXIS = 0;
    public static final int X_BUTTON = 1;
    public static final int A_BUTTON = 2;
    public static final int B_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int LEFT_TRIGGER_BUTTON = 7;
    public static final int RIGHT_TRIGGER_BUTTON = 8;
    public static final int PREV_BUTTON = 9;
    public static final int START_BUTTON = 10;
    public static final int LEFT_STICK_BUTTON = 11;
    public static final int RIGHT_STICK_BUTTON = 12;

    // Spark MAX CAN IDs //
    public static final int LEFT_FRONT_DRIVE_MOTOR_ID = 3; // NEO motor
    public static final int RIGHT_FRONT_DRIVE_MOTOR_ID = 2; // NEO motor
    public static final int LEFT_REAR_DRIVE_MOTOR_ID = 9; // NEO motor
    public static final int RIGHT_REAR_DRIVE_MOTOR_ID = 4; // NEO motor
    public static final int INTAKE_BAR_MOTOR_ID = 11; // NEO 550 motor
    public static final int INTAKE_ARM_MOTOR_ID = 6; // NEO motor
    public static final int ELEVATOR_STAGE_1_MOTOR_ID = 12; // NEO motor
    public static final int ELEVATOR_STAGE_2_MOTOR_ID = 10; // NEO 550 motor
    public static final int END_EFFECTOR_WHEEL_MOTOR_ID = 7; // NEO 550 motor
    public static final int END_EFFECTOR_ARM_MOTOR_ID = 8; // NEO 550 motor

    // Servo IDs //
    public static final int ELEVATOR_DROP_MOTOR_ID = 0;

    // PWM Ports //
    public static final int LED_PWM_ID = 4;
    
    // DIO (Digital Input/Output) Channels //
    // Example: public static final int RIGHT_ENCODER_CHANNEL_A = 0;
    // Example: public static final int RIGHT_ENCODER_CHANNEL_B = 1;
    // Example: public static final int LEFT_ENCODER_CHANNEL_A = 2;
    // Example: public static final int LEFT_ENCODER_CHANNEL_B = 3;

    // Drivetrain Constants //
    public static final double DRIVE_GEAR_RATIO = 10;
    public static final boolean REVERSE_LEFT_FRONT_MOTOR = true;
    public static final boolean REVERSE_LEFT_BACK_MOTOR = true;
    public static final boolean REVERSE_RIGHT_FRONT_MOTOR = false;
    public static final boolean REVERSE_RIGHT_BACK_MOTOR = false;
    public static final double GYRO_TURN_KP = 0.007; // P (Proportional) constant of a PID loop
    public static final double TRACKED_TAG_ROATION_KP = 0.3; // P (Proportional) constant of a PID loop
    public static final double TRACKED_TAG_FORWARD_DRIVE_KP = 0.4; // P (Proportional) constant of a PID loop
    public static final double TRACKED_TAG_STRAFE_DRIVE_KP = 0.5; // P (Proportional) constant of a PID loop
    public static final double APRILTAG_ROTATION_POWER_CAP = 0.3;
    public static final double APRILTAG_FORWARD_POWER_CAP = 0.3;
    public static final double APRILTAG_STRAFE_POWER_CAP = 0.3;
    public static final double APRILTAG_TRACKING_DISTANCE_THRESHOLD = 0.1;
    public static final double TURNING_THRESHOLD_DEGREES = 1;
    public static final double MAX_POWER_GYRO = 0.1;
    public static final double kP_FRONT_RIGHT_VELOCITY = 0.0010269;
	public static final double kP_FRONT_LEFT_VELOCITY = 0.0010269;
	public static final double kP_BACK_RIGHT_VELOCITY = 0.0010269;
	public static final double kP_BACK_LEFT_VELOCITY = 0.0010269;
	public static final double kP_X_CONTROLLER = 9.6421;
    public static final double kP_Y_CONTROLLER = 9.6421;
    public static final double kP_THETA_CONTROLLER = 9.6421;
	public static final double kMAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2*Math.PI;
	public static final double kMAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2*Math.PI;
    public static final double SLEW_RATE = 3;

    // Coral Elevator Constants //
    public static final boolean ELEVATOR_STAGE_1_INVERT = true;
    public static final boolean ELEVATOR_STAGE_2_INVERT = true;
    public static final boolean ELEVATOR_WHEEL_INVERT = true;
    public static final boolean ELEVATOR_ARM_INVERT = true;
    public static final double ELEVATOR_SPEED = 0.2;
    public static final double ARM_SPEED = 0.2;
    public static final double WHEEL_SPEED = 0.6;
    public static final double ARM_GRAVITY_CONST = -0.03;
    public static final int ELEVATOR_ROTATIONS_PER_INCH = 13; // Number of rotations elevator climb motor must complete to raise/lower elevator by one inch

    // Intake Constants //
    public static final double INTAKE_LIFT_GEAR_RATIO = 3*7*7*48/29;
    
    public static final double INTAKE_ARM_MAX_POWER = 0.25;
    public static final double INTAKE_ARM_MIN_POWER = 0.05;
    public static final double INTAKE_ARM_kP = 0.035;

    public static final double INTAKE_DEPLOY_LIMIT = 51;
    public static final double INTAKE_RETURN_LIMIT = 0;

    public static final boolean INTAKE_ARM_INVERT = true;
    public static final double DEPLOY_SPEED = 0.1;
    public static final boolean INTAKE_BAR_INVERT = false;
    public static final double INTAKE_BAR_SPEED = 0.8;

    public static final double PICK_UP_ALGAE_POSITION = 33;
    public static final double HOLD_ALGAE_POSITION = 2.0;
    public static final double PICK_UP_CORAL_POSITION = 53;
    public static final double HOLD_CORAL_POSITION = 24;
    
    public static final double GRAVITY_RESISTANCE = 0.05;

    // REV PH Channels //
    // Example: public static final int EXTENSION_SOLENOID_ID = 0;

    // Rev PDH Constants //
    public static final int LEFT_FRONT_DRIVE_MOTOR_PDH_CHANNEL = 11; // TODO: Fix this number with the correct channel
    public static final int RIGHT_FRONT_DRIVE_MOTOR_PDH_CHANNEL = 10; // TODO: Fix this number with the correct channel
    public static final int LEFT_BACK_DRIVE_MOTOR_PDH_CHANNEL = 12; // TODO: Fix this number with the correct channel
    public static final int RIGHT_BACK_DRIVE_MOTOR_PDH_CHANNEL = 13; // TODO: Fix this number with the correct channel
    public static final int INTAKE_BAR_MOTOR_PDH_CHANNEL = 1; // TODO: Fix this number with the correct channel
    public static final int INTAKE_DEPLOY_MOTOR_PDH_CHANNEL = 2; // TODO: Fix this number with the correct channel
    public static final int ELEVATOR_STAGE_1_MOTOR_PDH_CHANNEL = 3; // TODO: Fix this number with the correct channel
    public static final int ELEVATOR_STAGE_2_MOTOR_PDH_CHANNEL = 4; // TODO: Fix this number with the correct channel
    public static final int ELEVATOR_ARM_MOTOR_PDH_CHANNEL = 5; // TODO: Fix this number with the correct channel
    public static final int ELEVATOR_WHEEL_MOTOR_PDH_CHANNEL = 6; // TODO: Fix this number with the correct channel

    // Pneumatics Constants //
    public static final int COMPRESSOR_CAN_ID = 7;

    // Apriltag Vision Constants //
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(7);
    public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(18.5);
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(18);
    public static final String USB_CAMERA_NAME = "Arducam_OV9782_USB_Camera";
}
