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

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  private Constants() {}

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
  
  public static final int INTAKE_BAR_MOTOR_ID = 11;
  public static final int INTAKE_ARM_MOTOR_ID = 6;
  public static final int ELEVATOR_STAGE_1_MOTOR_ID = 12;
  public static final int ELEVATOR_STAGE_2_MOTOR_ID = 10;
  public static final int END_EFFECTOR_WHEEL_MOTOR_ID = 7;
  public static final int END_EFFECTOR_ARM_MOTOR_ID = 8;

  // ==================== SERVO / PWM ====================
  
  public static final int ELEVATOR_DROP_SERVO_ID = 0;
  public static final int LED_PWM_ID = 4;

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

  public class AprilTagFieldConstants {

        public static final double FIELD_LENGTH = 16.513048;
        public static final double FIELD_WIDTH = 8.042656;

        public static final List<AprilTag> TAGS = List.of(
        new AprilTag(1, new Pose3d(
                new Translation3d(11.878, 7.425, 0.889),
                new Rotation3d(0, 0, Math.toRadians(180)))),
        
        new AprilTag(2, new Pose3d(
                new Translation3d(11.915, 4.638, 1.124),
                new Rotation3d(0, 0, Math.toRadians(90)))),
        
        new AprilTag(3, new Pose3d(
                new Translation3d(11.312, 4.390, 1.124),
                new Rotation3d(0, 0, Math.toRadians(180)))),
        
        new AprilTag(4, new Pose3d(
                new Translation3d(11.312, 4.035, 1.124),
                new Rotation3d(0, 0, Math.toRadians(180)))),
        
        new AprilTag(5, new Pose3d(
                new Translation3d(11.915, 3.431, 1.124),
                new Rotation3d(0, 0, Math.toRadians(270)))),
        
        new AprilTag(6, new Pose3d(
                new Translation3d(11.878, 0.644, 0.889),
                new Rotation3d(0, 0, Math.toRadians(180)))),
        
        new AprilTag(7, new Pose3d(
                new Translation3d(11.953, 0.644, 0.889),
                new Rotation3d(0, 0, Math.toRadians(0)))),
        
        new AprilTag(8, new Pose3d(
                new Translation3d(12.271, 3.431, 1.124),
                new Rotation3d(0, 0, Math.toRadians(270)))),
        
        new AprilTag(9, new Pose3d(
                new Translation3d(4.022, 3.679, 1.124),
                new Rotation3d(0, 0, Math.toRadians(0)))),
        
        new AprilTag(10, new Pose3d(
                new Translation3d(12.519, 4.035, 1.124),
                new Rotation3d(0, 0, Math.toRadians(0)))),
        
        new AprilTag(11, new Pose3d(
                new Translation3d(12.271, 4.638, 1.124),
                new Rotation3d(0, 0, Math.toRadians(90)))),
        
        new AprilTag(12, new Pose3d(
                new Translation3d(11.953, 7.425, 0.889),
                new Rotation3d(0, 0, Math.toRadians(0)))),
        
        new AprilTag(13, new Pose3d(
                new Translation3d(16.533, 7.403, 0.552),
                new Rotation3d(0, 0, Math.toRadians(180)))),
        
        new AprilTag(14, new Pose3d(
                new Translation3d(16.533, 6.972, 0.552),
                new Rotation3d(0, 0, Math.toRadians(180)))),
        
        new AprilTag(15, new Pose3d(
                new Translation3d(16.533, 4.324, 0.552),
                new Rotation3d(0, 0, Math.toRadians(180)))),
        
        new AprilTag(16, new Pose3d(
                new Translation3d(16.533, 3.892, 0.552),
                new Rotation3d(0, 0, Math.toRadians(180)))),
        
        new AprilTag(17, new Pose3d(
                new Translation3d(4.663, 0.644, 0.889),
                new Rotation3d(0, 0, Math.toRadians(0)))),
        
        new AprilTag(18, new Pose3d(
                new Translation3d(4.626, 3.431, 1.124),
                new Rotation3d(0, 0, Math.toRadians(270)))),
        
        new AprilTag(19, new Pose3d(
                new Translation3d(5.229, 3.679, 1.124),
                new Rotation3d(0, 0, Math.toRadians(0)))),
        
        new AprilTag(20, new Pose3d(
                new Translation3d(5.229, 4.035, 1.124),
                new Rotation3d(0, 0, Math.toRadians(90)))),
        
        new AprilTag(21, new Pose3d(
                new Translation3d(4.626, 4.638, 1.124),
                new Rotation3d(0, 0, Math.toRadians(90)))),
        
        new AprilTag(22, new Pose3d(
                new Translation3d(4.663, 7.425, 0.889),
                new Rotation3d(0, 0, Math.toRadians(0)))),
        
        new AprilTag(23, new Pose3d(
                new Translation3d(4.588, 7.425, 0.889),
                new Rotation3d(0, 0, Math.toRadians(180)))),
        
        new AprilTag(24, new Pose3d(
                new Translation3d(4.271, 4.638, 1.124),
                new Rotation3d(0, 0, Math.toRadians(90)))),
        
        new AprilTag(25, new Pose3d(
                new Translation3d(4.022, 4.390, 1.124),
                new Rotation3d(0, 0, Math.toRadians(180)))),
        
        new AprilTag(26, new Pose3d(
                new Translation3d(4.022, 4.035, 1.124),
                new Rotation3d(0, 0, Math.toRadians(180)))),
        
        new AprilTag(27, new Pose3d(
                new Translation3d(4.271, 3.431, 1.124),
                new Rotation3d(0, 0, Math.toRadians(270)))),
        
        new AprilTag(28, new Pose3d(
                new Translation3d(4.588, 0.644, 0.889),
                new Rotation3d(0, 0, Math.toRadians(180)))),
        
        new AprilTag(29, new Pose3d(
                new Translation3d(0.008, 0.666, 0.552),
                new Rotation3d(0, 0, Math.toRadians(0)))),
        
        new AprilTag(30, new Pose3d(
                new Translation3d(0.008, 1.098, 0.552),
                new Rotation3d(0, 0, Math.toRadians(0)))),
        
        new AprilTag(31, new Pose3d(
                new Translation3d(0.008, 3.746, 0.552),
                new Rotation3d(0, 0, Math.toRadians(0)))),
        
        new AprilTag(32, new Pose3d(
                new Translation3d(0.008, 4.178, 0.552),
                new Rotation3d(0, 0, Math.toRadians(0))))
        );

}
}