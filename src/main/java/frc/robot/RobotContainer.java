package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.MecanumDriveCommands;
import frc.robot.subsystems.Flywheel.Flywheel;
import frc.robot.subsystems.Flywheel.FlywheelIO;
import frc.robot.subsystems.Flywheel.FlywheelIOReal;
import frc.robot.subsystems.Gyro.GyroIO;
import frc.robot.subsystems.Gyro.GyroIONavX;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hood.HoodIOReal;
import frc.robot.subsystems.Turntable.Turntable;
import frc.robot.subsystems.Turntable.TurntableIOReal;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.drive.MecanumDrive.MecanumDrive;
import frc.robot.subsystems.drive.MecanumDrive.MecanumModuleIO;
import frc.robot.subsystems.drive.MecanumDrive.MecanumModuleIOSpark;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final MecanumDrive drive;
  // private final Turret turret;
  // private final Flywheel flywheel;
  private final Hood hood;
  // private final Turntable turntable;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
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
                new GyroIONavX());

        // flywheel = new Flywheel(new FlywheelIOReal());
        hood = new Hood(new HoodIOReal());
        // turntable = new Turntable(new TurntableIOReal());
        // turret = new Turret(flywheel, hood, turntable);

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

        // flywheel = null;
        hood = null;
        // turntable = null;
        // turret = null;
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

        // flywheel = null;
        hood = null;
        // turntable = null;
        // turret = null;
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

    //controller.leftTrigger().onTrue(flywheel.StopFlywheelsCommand());
    //controller.rightTrigger().onTrue(flywheel.RunTuneableFlywheelsCommand());

    controller.povUp().whileTrue(hood.incrementHoodAngleCommand());
    controller.povDown().whileTrue(hood.decrementHoodAngleCommand());

    // controller.povRight().whileTrue(turntable.incrementTurntableAngleCommand());
    // controller.povLeft().whileTrue(turntable.decrementTurntableAngleCommand());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}