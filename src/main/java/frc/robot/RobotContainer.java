package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.MecanumDriveCommands;
import frc.robot.subsystems.Flywheel.Flywheel;
import frc.robot.subsystems.Flywheel.FlywheelIO;
import frc.robot.subsystems.Flywheel.FlywheelIOReal;
import frc.robot.subsystems.Gyro.GyroIO;
import frc.robot.subsystems.Gyro.GyroIONavX;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Intake.IntakeIOSpark;
import frc.robot.subsystems.Intake.IntakeIOSparkOnboardPID;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIOReal;
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
  // private final MecanumDrive drive;
  private final Indexer indexer;
  private final Intake intake;
  private final MecanumDrive drive;
  // private final Turret turret;
  private final Flywheel flywheel;
  private final Hood hood;
  // private final Turntable turntable;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);


  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new MecanumDrive(
                new MecanumModuleIO[] {
                  new MecanumModuleIOSpark(0), // FL
                  new MecanumModuleIOSpark(1), // FR
                  new MecanumModuleIOSpark(2), // BL
                  new MecanumModuleIOSpark(3)  // BR
                },
                new GyroIONavX());

        flywheel = new Flywheel(new FlywheelIOReal());
        hood = new Hood(new HoodIOReal());
        // turntable = new Turntable(new TurntableIOReal());
        // turret = new Turret(flywheel, hood, turntable);

        break;

      case SIM:
        // Sim robot - use empty IO implementations
        // drive =
        //     new MecanumDrive(
        //         new MecanumModuleIO[] {
        //           new MecanumModuleIO() {},
        //           new MecanumModuleIO() {},
        //           new MecanumModuleIO() {},
        //           new MecanumModuleIO() {}
        //         },
        //         new GyroIO() {});
        drive =
            new MecanumDrive(
                new MecanumModuleIO[] {
                  new MecanumModuleIO() {},
                  new MecanumModuleIO() {},
                  new MecanumModuleIO() {},
                  new MecanumModuleIO() {}
                },
                new GyroIO() {});

        flywheel = null;
        hood = null;
        // turntable = null;
        // turret = null;
        break;

      default:
        // Replayed robot, disable IO implementations
        // drive =
        //     new MecanumDrive(
        //         new MecanumModuleIO[] {
        //           new MecanumModuleIO() {},
        //           new MecanumModuleIO() {},
        //           new MecanumModuleIO() {},
        //           new MecanumModuleIO() {}
        //         },
        //         new GyroIO() {});
        drive =
            new MecanumDrive(
                new MecanumModuleIO[] {
                  new MecanumModuleIO() {},
                  new MecanumModuleIO() {},
                  new MecanumModuleIO() {},
                  new MecanumModuleIO() {}
                },
                new GyroIO() {});

        flywheel = null;
        hood = null;
        // turntable = null;
        // turret = null;
        break;
    }

    intake = new Intake(new IntakeIOSparkOnboardPID());
    indexer = new Indexer(new IndexerIOReal());

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

      //controller.button(5).onTrue(intake.setIntake(Units.degreesToRadians(19.44)));
      //controller.button(6).onTrue(intake.setIntake(Units.degreesToRadians(118)));
      // controller.rightBumper().onTrue(new InstantCommand(()->intake.setIntakeDoorPosition(Units.degreesToRadians(19.44)), intake));
      // controller.leftBumper().onTrue(new SetIntakeHigh(intake));
      // controller.rightTrigger().onTrue(new RunCommand(()-> intake.stopIntake(), intake));
      controller.leftBumper().whileTrue(new RunCommand(()-> intake.runIntake(controller.getLeftTriggerAxis()), intake)).
                                            onFalse(new InstantCommand(()-> intake.runIntake(0.0), intake));

    controller.rightBumper().whileTrue(indexer.runIndexerCommandDutyCycle());

    controller.leftTrigger().onTrue(flywheel.decrementRpmSetPoint());
    controller.rightTrigger().onTrue(flywheel.incrementRpmSetPoint());
    controller.b().onTrue(flywheel.StopFlywheelsCommand());

    controller.povUp().whileTrue(hood.incrementHoodAngleCommand());
    controller.povDown().whileTrue(hood.decrementHoodAngleCommand());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}