package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.drive.MecanumDrive.MecanumDrive;

/**
 * Command that continuously tracks the hub using vision-estimated pose
 * and aims the turret accordingly.
 */
public class AutoTrackHub extends Command {
    
    private final Turret turret;
    private final MecanumDrive drivetrain;

    /**
     * Creates a new AutoTrackHub command.
     * 
     * @param turret The turret subsystem to aim
     * @param drivetrain The drivetrain for pose estimation
     */
    public AutoTrackHub(Turret turret, MecanumDrive drivetrain) {
        this.turret = turret;
        this.drivetrain = drivetrain;
        
        // Declare subsystem requirements
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        // Nothing specific needed on init
    }

    @Override
    public void execute() {
        // Get the current estimated pose from the drivetrain (which includes vision corrections)
        Pose2d currentPose = drivetrain.getEstimatedPose();
        
        // Use the turret's auto-aim function with the current robot pose
        turret.autoAimTurret(
            currentPose.getX(),
            currentPose.getY(),
            currentPose.getRotation().getRadians()
        );
    }

    @Override
    public void end(boolean interrupted) {
        // Optionally stop the turret or leave it at current position
    }

    @Override
    public boolean isFinished() {
        // This command runs until interrupted (e.g., button released)
        return false;
    }
}