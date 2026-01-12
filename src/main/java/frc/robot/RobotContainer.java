package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.MecanumDriveCommands;
import frc.robot.subsystems.Gyro.GyroIO;
import frc.robot.subsystems.Gyro.SimpleNavX;
import frc.robot.subsystems.Gyro.SimpleNavXStandalone;
import frc.robot.subsystems.drive.MecanumDrive.MecanumDrive;
import frc.robot.subsystems.drive.MecanumDrive.MecanumModuleIO;
import frc.robot.subsystems.drive.MecanumDrive.MecanumModuleIOSpark;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final MecanumDrive drive;
  
  // Standalone NavX for direct access (useful for testing/debugging)
  private final SimpleNavXStandalone navxStandalone;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    // Initialize standalone NavX (for direct access and testing)
    if (Constants.currentMode == Constants.Mode.REAL) {
      navxStandalone = new SimpleNavXStandalone();
    } else {
      navxStandalone = null;
    }

    switch (Constants.currentMode) {
      case REAL:
        // Real robot with NavX and Spark MAX mecanum modules
        drive =
            new MecanumDrive(
                new MecanumModuleIO[] {
                  new MecanumModuleIOSpark(0), // FL
                  new MecanumModuleIOSpark(1), // FR
                  new MecanumModuleIOSpark(2), // BL
                  new MecanumModuleIOSpark(3)  // BR
                },
                new SimpleNavX());
        break;

      case SIM:
        // Sim robot - use empty IO implementations
        drive =
            new MecanumDrive(
                new MecanumModuleIO[] {
                  new MecanumModuleIO() {},
                  new MecanumModuleIO() {},
                  new MecanumModuleIO() {},
                  new MecanumModuleIO() {}
                },
                new GyroIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new MecanumDrive(
                new MecanumModuleIO[] {
                  new MecanumModuleIO() {},
                  new MecanumModuleIO() {},
                  new MecanumModuleIO() {},
                  new MecanumModuleIO() {}
                },
                new GyroIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    autoChooser.addDefaultOption("None", Commands.none());

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default command: field-relative mecanum drive with squared inputs
    drive.setDefaultCommand(
        MecanumDriveCommands.joystickDriveFieldRelative(
            drive,
            () -> -controller.getLeftY(),  // Forward/back
            () -> -controller.getLeftX(),  // Strafe
            () -> -controller.getRightX())); // Rotation

    // Robot-relative drive when A button is held
    controller
        .a()
        .whileTrue(
            MecanumDriveCommands.joystickDriveRobotRelative(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()));

    // Snap to 0° when Y button is held
    controller
        .y()
        .whileTrue(
            MecanumDriveCommands.snapToAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                new Rotation2d(0)));

    // Stop when X button is pressed
    controller.x().onTrue(MecanumDriveCommands.stop(drive));

    // Reset heading when B button is pressed
    controller.b().onTrue(MecanumDriveCommands.resetHeading(drive));

    // === Standalone NavX Test Functions ===
    if (navxStandalone != null) {
      // Zero gyro with Start button
      controller.start().onTrue(Commands.runOnce(() -> {
        navxStandalone.zeroYaw();
        System.out.println("Standalone NavX zeroed!");
      }));

      // Print gyro info with Back button
      controller.back().onTrue(Commands.runOnce(() -> {
        System.out.println("=== Standalone NavX Status ===");
        System.out.println("Connected: " + navxStandalone.isConnected());
        System.out.println("Raw Yaw: " + navxStandalone.getRawYaw() + "°");
        System.out.println("Adjusted Yaw: " + navxStandalone.getYawDegrees() + "°");
        System.out.println("Rotation2d: " + navxStandalone.getRotation2d());
        System.out.println("Velocity: " + navxStandalone.getYawVelocityDegreesPerSec() + "°/s");
        System.out.println("==============================");
      }));

      // Set yaw to 90° with left bumper (for testing)
      controller.leftBumper().onTrue(Commands.runOnce(() -> {
        navxStandalone.setYaw(90);
        System.out.println("Standalone NavX set to 90°");
      }));

      // Set yaw to -90° with right bumper (for testing)
      controller.rightBumper().onTrue(Commands.runOnce(() -> {
        navxStandalone.setYaw(-90);
        System.out.println("Standalone NavX set to -90°");
      }));
    }
  }

  /**
   * Gets the standalone NavX instance for direct access.
   * @return The SimpleNavXStandalone instance, or null if not in REAL mode
   */
  public SimpleNavXStandalone getNavxStandalone() {
    return navxStandalone;
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}