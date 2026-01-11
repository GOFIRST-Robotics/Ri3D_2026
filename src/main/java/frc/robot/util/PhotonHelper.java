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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;

public class PhotonHelper {
    static PhotonCamera camera = new PhotonCamera("Arducam_OV9782_USB_Camera");

    static final AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(Constants.AprilTagFieldConstants.TAGS, 6.5012, 3.1664);
    static final Transform3d zeroOffset = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));
    static final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, zeroOffset);

    public static Optional<EstimatedRobotPose> getCameraFieldPos() {  return poseEstimator.update(camera.getLatestResult()); }
    

    public static boolean cameraConnected() { return camera.isConnected(); }
}