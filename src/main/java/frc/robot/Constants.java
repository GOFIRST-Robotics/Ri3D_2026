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
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  private Constants() {}

  public static final double TWO_PI = 2 * Math.PI;

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

  // ==================== DRIVETRAIN CONSTANTS ====================
  
  /** Wheel diameter in meters (6 inch wheels). */
  public static final double WHEEL_DIAMETER = Units.inchesToMeters(6);
  
  /** Wheel radius in meters. */
  public static final double WHEEL_RADIUS = WHEEL_DIAMETER / 2.0;
  
  /** Distance between centers of right and left wheels (meters). */
  public static final double TRACK_WIDTH = Units.inchesToMeters(21);
  
  /** Distance between centers of front and back wheels (meters). */
  public static final double WHEEL_BASE = Units.inchesToMeters(16.75);
  
  /** Gear ratio of the drivetrain (motor rotations per wheel rotation). */
  public static final double DRIVE_GEAR_RATIO = 10.0;

  // ==================== MECANUM DRIVE CONSTANTS ====================
  
  public static final class MecanumConstants {
    private MecanumConstants() {}

    // CAN IDs (FL=1, FR=2, BL=3, BR=4)
    public static final int frontLeftCanId = 11;
    public static final int frontRightCanId = 12;
    public static final int backLeftCanId = 13;
    public static final int backRightCanId = 14;

    // Motor inversions
    public static final boolean frontLeftInverted = false;
    public static final boolean frontRightInverted = true;
    public static final boolean backLeftInverted = false;
    public static final boolean backRightInverted = true;

    // Current limit
    public static final int driveMotorCurrentLimit = 60;

    // Encoder conversion factors (motor rotations -> wheel radians)
    public static final double encoderPositionFactorRad = (2.0 * Math.PI) / DRIVE_GEAR_RATIO;
    public static final double encoderVelocityFactorRadPerSec = (2.0 * Math.PI) / 60.0 / DRIVE_GEAR_RATIO;

    // Closed-loop PID gains
    public static final double kP = 0.01;
    public static final double kD = 0.0;

    // Feedforward gains (V = kS*sign(ω) + kV*ω)
    public static final double kS = 0.0;
    public static final double kV = 0.0;
  }

  // ==================== OTHER MOTOR CAN IDs ====================
  
  // public static final int INTAKE_BAR_MOTOR_ID = 11;
  // public static final int INTAKE_ARM_MOTOR_ID = 6;
  // public static final int ELEVATOR_STAGE_1_MOTOR_ID = 12;
  // public static final int ELEVATOR_STAGE_2_MOTOR_ID = 10;
  // public static final int END_EFFECTOR_WHEEL_MOTOR_ID = 7;
  // public static final int END_EFFECTOR_ARM_MOTOR_ID = 8;
  public static final int TURRET_LEFT_FLYWHEEL_MOTOR_ID = 6;
  public static final int TURRET_RIGHT_FLYWHEEL_MOTOR_ID = 10;
  public static final int TURRET_HOOD_MOTOR_ID = 20;

  // ==================== SERVO / PWM ====================
  
  public static final int ELEVATOR_DROP_SERVO_ID = 0;
  public static final int LED_PWM_ID = 4;

  // ==================== TURRET CONSTANTS ====================

  public static final class TurretConstants {
    public static final double TURRET_HOOD_GEAR_RATIO = 475.0*10.0/85.0;
    public static final double TURRET_TURNTABLE_GEAR_RATIO = 1;

    public static final double FLYWHEEL_DIAMETER = 1;
    public static final double FLYWHEEL_BALL_COMPRESSION = 0.25;
    
    public static final double TURRET_LOCAL_POS_X = 0; // meters, right hand rule!!
    public static final double TURRET_LOCAL_POS_Y = 0; // meters, right hand rule!!

    public static final double TURRET_HOOD_MOTOR_MIN_ROTATIONS = 0;
    public static final double TURRET_HOOD_MOTOR_MAX_ROTATIONS = 8;
    public static final double TURRET_HOOD_MIN_RADIANS = Units.degreesToRadians(30);
    public static final double TURRET_HOOD_MAX_RADIANS = Units.degreesToRadians(80);
    public static final double TURRET_HOOD_RANGE_RADIANS = TURRET_HOOD_MAX_RADIANS - TURRET_HOOD_MIN_RADIANS;

    public static final double TURRET_TURNTABLE_MOTOR_MIN_ROTATIONS = 5;
    public static final double TURRET_TURNTABLE_MOTOR_MAX_ROTATIONS = 5;
    public static final double TURRET_TURNTABLE_MAX_RADIANS = Units.degreesToRadians(175);

    public static final double TURRET_TURNTABLE_CHANGE_SPEED = 0.015;
    public static final double TURRET_HOOD_CHANGE_SPEED = 0.01;

    public static final double TURRET_FLYWHEEL_ACCEPTABLE_FLYWHEEL_RPM_ERROR = 100;
    public static final double TURRET_HOOD_ACCEPTABLE_RADIAN_ERROR = Units.degreesToRadians(2);
    public static final double TURRET_TURNTABLE_ACCEPTABLE_RADIAN_ERROR = Units.degreesToRadians(2);

    public static final double GRAVITY_CONSTANT = -9.81;

    public static final double TURRET_VERTICAL_DISTANCE_TO_GOAL = 1.8288;
    public static final double TURRET_VERTICAL_DISTANCE_APEX_OFFSET = 0.254;
    public static final double TURRET_TIME_INTO_GOAL_AFTER_APEX = Math.sqrt(-TURRET_VERTICAL_DISTANCE_APEX_OFFSET / GRAVITY_CONSTANT);
    public static final double GOAL_FIELD_SPACE_X_POSITION = 10;
    public static final double GOAL_FIELD_SPACE_Y_POSITION = -10;

    // Turntable PIDs
    public static final double TURNTABLE_kP = 0.0;
    public static final double TURNTABLE_kI = 0.0;
    public static final double TURNTABLE_kD = 0.0;
    public static final double TURNTABLE_kS = 0.0;
    public static final double TURNTABLE_kV = 0.0;
    public static final double TURNTABLE_kA = 0.0;
    public static final double TURNTABLE_CRUISE_VEL = 0.0;
    public static final double TURNTABLE_MAX_ACCEL = 0.0;
    public static final double TURNTABLE_ALLOWED_ERROR = 0.5;

    // Hood PIDs
    public static final double HOOD_kP = 0.4;
    public static final double HOOD_kI = 0.0;
    public static final double HOOD_kD = 0.0;
    public static final double HOOD_kV = 0.0;
    public static final double HOOD_kCos = 0.8075;
    public static final double HOOD_CRUISE_VEL = (150 * Constants.TurretConstants.TURRET_HOOD_GEAR_RATIO / 360) * 60;
    public static final double HOOD_MAX_ACCEL = 10000000; // it actually needs to be this...
    public static final double HOOD_ALLOWED_PROFILE_ERROR = 0.025;

    // Flywheel PIDs
    public static final double TOP_FLYWHEEL_KP = 0.0001;
    public static final double TOP_FLYWHEEL_KI = 0.0;
    public static final double TOP_FLYWHEEL_KD = 0.0;
    public static final double TOP_FLYWHEEL_KV = 0.002275;
    public static final double TOP_FLYWHEEL_ACCEL = 4000.0;

    public static final double BOTTOM_FLYWHEEL_KP = 0.0001;
    public static final double BOTTOM_FLYWHEEL_KI = 0.0;
    public static final double BOTTOM_FLYWHEEL_KD = 0.0;
    public static final double BOTTOM_FLYWHEEL_KV = 0.002275;
    public static final double BOTTOM_FLYWHEEL_ACCEL = 12000.0;

  }

  // ==================== ELEVATOR CONSTANTS ====================
  
  public static final boolean ELEVATOR_STAGE_1_INVERT = true;
  public static final boolean ELEVATOR_STAGE_2_INVERT = true;
  public static final boolean ELEVATOR_WHEEL_INVERT = true;
  public static final boolean ELEVATOR_ARM_INVERT = true;
  public static final double ELEVATOR_SPEED = 0.2;
  public static final double ARM_SPEED = 0.2;
  public static final double WHEEL_SPEED = 0.6;
  public static final double ARM_GRAVITY_CONST = -0.03;
  public static final int ELEVATOR_ROTATIONS_PER_INCH = 13;

  // ==================== INTAKE CONSTANTS ====================
  
  public static final double INTAKE_LIFT_GEAR_RATIO = 3 * 7 * 7 * 48 / 29;
  public static final double INTAKE_ARM_MAX_POWER = 0.25;
  public static final double INTAKE_ARM_MIN_POWER = 0.05;
  public static final double INTAKE_ARM_kP = 0.035;
  public static final double INTAKE_DEPLOY_LIMIT = 51;
  public static final double INTAKE_RETURN_LIMIT = 0;
  public static final boolean INTAKE_ARM_INVERT = true;
  public static final double DEPLOY_SPEED = 0.1;
  public static final boolean INTAKE_BAR_INVERT = false;
  public static final double INTAKE_BAR_SPEED = 0.8;

  // Intake positions
  public static final double PICK_UP_ALGAE_POSITION = 33;
  public static final double HOLD_ALGAE_POSITION = 2.0;
  public static final double PICK_UP_CORAL_POSITION = 53;
  public static final double HOLD_CORAL_POSITION = 24;
  public static final double GRAVITY_RESISTANCE = 0.05;

  // ==================== VISION CONSTANTS ====================
  
  public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(7);
  public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(18.5);
  public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(18);
  public static final String USB_CAMERA_NAME = "Arducam_OV9782_USB_Camera";
}