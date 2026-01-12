package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Gyro.SimpleNavXStandalone;

public class RobotContainer {

  // 1. Define ONLY the subsystem we want to test
  private final SimpleNavXStandalone navx = new SimpleNavXStandalone();

  // 2. Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    configureButtonBindings();
    
    // 3. CONTINUOUS LOGGING
    // This command runs every loop (20ms) effectively acting as a telemetry printer.
    // Open SmartDashboard to see these values update live.
    navx.setDefaultCommand(Commands.run(() -> {
        SmartDashboard.putBoolean("NavX/Connected", navx.isConnected());
        SmartDashboard.putNumber("NavX/Raw Yaw", navx.getRawYaw());
        SmartDashboard.putNumber("NavX/Adjusted Yaw", navx.getYawDegrees());
        SmartDashboard.putNumber("NavX/Velocity DegPerSec", navx.getYawVelocityDegreesPerSec());
        SmartDashboard.putString("NavX/Rotation2d", navx.getRotation2d().toString());
    }, navx));
  }

  private void configureButtonBindings() {
    // Press 'A' to Zero the Gyro
    controller.a().onTrue(Commands.runOnce(() -> {
        navx.zeroYaw();
        System.out.println("NavX Zeroed via Controller");
    }, navx));

    // Press 'B' to artificially set the angle to 90 (to test offset logic)
    controller.b().onTrue(Commands.runOnce(() -> {
        navx.setYaw(90);
        System.out.println("NavX set to 90 degrees");
    }, navx));
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}