package frc.robot.subsystems.Vision;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class VisionIOPhoton implements VisionIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  private final AprilTagFieldLayout aprilTagFieldLayout;
  private final Supplier<Rotation2d> gyroHeadingSupplier;

  public VisionIOPhoton(String cameraName, Transform3d robotToCamera, Supplier<Rotation2d> gyroHeadingSupplier) {
    camera = new PhotonCamera(cameraName);
    this.gyroHeadingSupplier = gyroHeadingSupplier;
    
    aprilTagFieldLayout = new AprilTagFieldLayout(
        Constants.AprilTagFieldConstants.TAGS,
        Constants.AprilTagFieldConstants.FIELD_LENGTH,
        Constants.AprilTagFieldConstants.FIELD_WIDTH);
    
    poseEstimator = new PhotonPoseEstimator(
        aprilTagFieldLayout,
        PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
        robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    Rotation2d heading2d = gyroHeadingSupplier.get();
    Rotation3d heading3d = new Rotation3d(0, 0, heading2d.getRadians());
    poseEstimator.addHeadingData(Timer.getFPGATimestamp(), heading3d);
    
    var result = camera.getLatestResult();
    Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
    
    
    if (estimatedPose.isPresent()) {
      EstimatedRobotPose est = estimatedPose.get();
      inputs.hasTarget = true;
      inputs.estimatedPose = est.estimatedPose;
      inputs.timestampSeconds = est.timestampSeconds;
      inputs.tagCount = est.targetsUsed.size();
      
      // Calculate average distance to tags
      double avgDist = 0;
      int[] ids = new int[est.targetsUsed.size()];
      for (int i = 0; i < est.targetsUsed.size(); i++) {
        var target = est.targetsUsed.get(i);
        ids[i] = target.getFiducialId();
        var tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
        if (tagPose.isPresent()) {
          avgDist += tagPose.get().toPose2d().getTranslation()
              .getDistance(est.estimatedPose.toPose2d().getTranslation());
        }
      }
      inputs.avgTagDistance = est.targetsUsed.size() > 0 ? avgDist / est.targetsUsed.size() : 0;
      inputs.tagIds = ids;
    } else {
      inputs.hasTarget = false;
      inputs.estimatedPose = new Pose3d();
      inputs.timestampSeconds = 0.0;
      inputs.tagCount = 0;
      inputs.avgTagDistance = 0.0;
      inputs.tagIds = new int[0];
    }
  }
}