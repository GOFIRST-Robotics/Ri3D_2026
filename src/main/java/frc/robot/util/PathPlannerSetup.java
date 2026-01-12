package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.MecanumDrive.MecanumDrive;

public class PathPlannerSetup {
    public static RobotConfig config;


    public static void initializedPathPlannerConfig() {
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
    } 

    public static void configureAutoBuilder(MecanumDrive mecanumDrive) {
        AutoBuilder.configure(
            mecanumDrive::getPose, 
            mecanumDrive::resetPose, 
            mecanumDrive::getChassisSpeeds, 
            mecanumDrive::runFieldRelative, 
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0)
            ), 
            config, 
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            mecanumDrive
        ); // Reference to this subsystem to set requirements
    }

}
