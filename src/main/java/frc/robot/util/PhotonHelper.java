package frc.robot.util;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PnpResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

public class PhotonHelper {
    static PhotonCamera camera = new PhotonCamera("Arducam_OV9782_USB_Camera");

    static final AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(Constants.AprilTagFieldConstants.TAGS, Constants.AprilTagFieldConstants.FIELD_LENGTH, Constants.AprilTagFieldConstants.FIELD_WIDTH);
    static final Transform3d zeroOffset = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));
    static final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, zeroOffset);

    public static Optional<EstimatedRobotPose> getCameraFieldPos() {  return poseEstimator.update(camera.getLatestResult()); }
        
    public static boolean cameraConnected() { return camera.isConnected(); }


    public static Pose3d RobotToCameraPose(Pose3d robotPose, double turretYawRadians) {
        // 1. Define where the turret is on the robot
        Transform3d robotToTurretCenter = new Transform3d(
            new Translation3d(TurretConstants.TURRET_LOCAL_POS_X, TurretConstants.TURRET_LOCAL_POS_Y, TurretConstants.TURRET_LOCAL_POS_Z), 
            new Rotation3d(0, 0, 0)
        );
            
        // 2. Define where the camera is relative to the spinning turret center, pitch doesn't change
        Transform3d turretToCamera = new Transform3d(
            new Translation3d(TurretConstants.CAMERA_RADIUS, 0, TurretConstants.CAMERA_HEIGHT), 
            new Rotation3d(0, 0, turretYawRadians)
        );

        // 3. Combine the two transforms to get robot to camera
        Transform3d robotToCamera = robotToTurretCenter.plus(turretToCamera);

        // 4. Convert to Pose3d
        return robotPose.transformBy(robotToCamera);
    }

    public static Pose3d CameraToRobotPose(Pose3d cameraPose, double turretYawRadians) {
        // 1. Define where the turret is on the robot
        Transform3d robotToTurretCenter = new Transform3d(
            new Translation3d(TurretConstants.TURRET_LOCAL_POS_X, TurretConstants.TURRET_LOCAL_POS_Y, TurretConstants.TURRET_LOCAL_POS_Z), 
            new Rotation3d(0, 0, 0)
        );
            
        // 2. Define where the camera is relative to the spinning turret center, pitch doesn't change
        Transform3d turretToCamera = new Transform3d(
            new Translation3d(TurretConstants.CAMERA_RADIUS, 0, TurretConstants.CAMERA_HEIGHT), 
            new Rotation3d(0, 0, turretYawRadians)
        );

        // 3. Combine the two transforms to get robot to camera
        Transform3d robotToCamera = robotToTurretCenter.plus(turretToCamera);
        
        // 4. Invert and apply to get robot pose from camera pose
        return cameraPose.transformBy(robotToCamera.inverse());
    }

}