package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.MecanumDriveCommands;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberIO;
import frc.robot.subsystems.Climber.ClimberIOReal;
import frc.robot.subsystems.Climber.ClimberIO.ClimbPosition;
import frc.robot.subsystems.Flywheel.Flywheel;
import frc.robot.subsystems.Flywheel.FlywheelIOReal;
import frc.robot.subsystems.Gyro.GyroIO;
import frc.robot.subsystems.Gyro.GyroIONavX;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOSparkOnboardPID;
import frc.robot.subsystems.Intake.IntakeIOSparkRIOPID;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIOReal;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hood.HoodIOReal;
import frc.robot.subsystems.Turntable.Turntable;
import frc.robot.subsystems.Turntable.TurntableIOReal;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionIOPhoton;
import frc.robot.subsystems.drive.MecanumDrive.MecanumDrive;
import frc.robot.subsystems.drive.MecanumDrive.MecanumModuleIO;
import frc.robot.subsystems.drive.MecanumDrive.MecanumModuleIOSpark;

import java.lang.invoke.ConstantCallSite;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Indexer indexer;
  private final Intake intake;
  private final MecanumDrive drive;
  private final Turret turret;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Turntable turntable;
  private final Climber climber;
  private final Vision vision;

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
        turntable = new Turntable(new TurntableIOReal(), drive);
        turret = new Turret(flywheel, hood, turntable);
        climber = new Climber(new ClimberIOReal());

        vision = new Vision(new VisionIOPhoton("Arducam_OV9782_USB_Camera", turntable::getDynamicCameraTransform, drive::getGyroYaw),drive);

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

        flywheel = null;
        hood = null;
        turntable = null;
        turret = null;
        climber = null;

        vision = null;
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

        flywheel = null;
        hood = null;
        turntable = null;
        turret = null;
        climber = null;

        vision = null;
        break;
    }

    intake = new Intake(new IntakeIOSparkRIOPID());
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
          () -> controller.getRightX())); // Rotation

    // INTAKE COMMANDS //
    controller.button(8).onTrue(intake.setIntake(Constants.IntakeConstants.INTAKE_DOOR_POSITION_DEPLOYED));
    controller.button(7).onTrue(intake.setIntake(Constants.IntakeConstants.INTAKE_DOOR_POSITION_STORED));
    /////////////////////

    controller.leftBumper().whileTrue(new RunCommand(()-> intake.runIntake(1), intake)).
                                          onFalse(new InstantCommand(()-> intake.runIntake(0.0), intake));

    controller.button(10).onTrue(MecanumDriveCommands.resetHeading(drive));

    controller.rightBumper().whileTrue(indexer.runIndexerCommandDutyCycle());

    controller.leftTrigger().whileTrue(flywheel.decrementRpmSetPoint());
    controller.rightTrigger().whileTrue(flywheel.incrementRpmSetPoint());
    controller.button(1).onTrue(flywheel.StopFlywheelsCommand());

    // TARGET OFFSET COMMANDS //
    controller.povUp().whileTrue(turret.offsetTargetPoint(0, Constants.TARGET_OFFSET_CHANGE_SPEED));
    controller.povDown().whileTrue(turret.offsetTargetPoint(0, -Constants.TARGET_OFFSET_CHANGE_SPEED));
    controller.povLeft().whileTrue(turret.offsetTargetPoint(-Constants.TARGET_OFFSET_CHANGE_SPEED, 0));
    controller.povRight().whileTrue(turret.offsetTargetPoint(Constants.TARGET_OFFSET_CHANGE_SPEED, 0));
    ////////////////////////////

    controller.button(2).whileTrue(turntable.incrementTurntableAngleCommand());
    controller.button(3).whileTrue(turntable.decrementTurntableAngleCommand());

    controller.button(4).toggleOnTrue(turntable.facePointCommand(new Translation2d(Constants.TurretConstants.RED_GOAL_FIELD_SPACE_X_POSITION, Constants.TurretConstants.RED_GOAL_FIELD_SPACE_Y_POSITION), vision));

    controller.button(9).toggleOnTrue(turret.aimAndShoot(Constants.TurretConstants.RED_GOAL_POSE, Constants.TurretConstants.SHOOT_APEX_OFFSET, vision, indexer, drive)).onFalse(flywheel.StopFlywheelsCommand());

    // controller.button(8).onTrue(climber.climbElevatorCommand(ClimbPosition.RUNG_ONE, true));
    // controller.button(7).onTrue(climber.climbElevatorCommand(ClimbPosition.ZERO, true));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}