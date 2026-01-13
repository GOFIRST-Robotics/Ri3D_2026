package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.MecanumDriveCommands;
import frc.robot.commands.SetIntakeHigh;
import frc.robot.commands.SetIntakeLow;
import frc.robot.subsystems.Gyro.GyroIO;
import frc.robot.subsystems.Gyro.GyroIONavX;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Intake.IntakeIOSpark;
import frc.robot.subsystems.drive.MecanumDrive.MecanumDrive;
import frc.robot.subsystems.drive.MecanumDrive.MecanumModuleIO;
import frc.robot.subsystems.drive.MecanumDrive.MecanumModuleIOSpark;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final MecanumDrive drive;

  private final Intake intake;

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
        
        intake = new Intake(new IntakeIOSpark());
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
        intake = new Intake(new IntakeIO() {});
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

        intake = new Intake(new IntakeIO() {});
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

      controller.rightBumper().onTrue(new SetIntakeLow(intake));
      controller.leftBumper().onTrue(new SetIntakeHigh(intake));
      controller.rightTrigger().onTrue(new RunCommand(()-> intake.stopIntake(), intake));


    // Stop when X button is pressed
    controller.x().onTrue(MecanumDriveCommands.stop(drive));

    // Reset heading when B button is pressed
    controller.b().onTrue(MecanumDriveCommands.resetHeading(drive));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}