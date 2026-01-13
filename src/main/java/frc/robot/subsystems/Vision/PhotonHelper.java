package frc.robot.subsystems.Vision;

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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.subsystems.drive.MecanumDrive.MecanumDrive;

public class PhotonHelper {
    static PhotonCamera camera = new PhotonCamera("Arducam_OV9782_USB_Camera");

    static final AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(Constants.AprilTagFieldConstants.TAGS, Constants.AprilTagFieldConstants.FIELD_LENGTH, Constants.AprilTagFieldConstants.FIELD_WIDTH);
    static final Transform3d zeroOffset = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));
    static final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, zeroOffset);

    public static Optional<EstimatedRobotPose> getCameraFieldPos() {  return poseEstimator.update(camera.getLatestResult()); }
        
    public static boolean cameraConnected() { return camera.isConnected(); }

    /**
     * Calculate standard deviations based on the estimated pose
     * @param estimatedPose The estimated robot pose from PhotonVision
     * @return Matrix of standard deviations [x, y, theta]
     */
    public static Matrix<N3, N1> getEstimationStdDevs(EstimatedRobotPose estimatedPose) {
        var estStdDevs = Constants.SINGLE_TAG_STD_DEVS;
        var targets = estimatedPose.targetsUsed;
        int numTags = targets.size();
        
        // If we see multiple tags, use tighter standard deviations
        if (numTags > 1) {
            estStdDevs = Constants.MULTI_TAG_STD_DEVS;
        }
        
        // Calculate average distance to tags
        double avgDist = 0;
        for (var target : targets) {
            var tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isPresent()) {
                avgDist += tagPose.get().toPose2d().getTranslation()
                    .getDistance(estimatedPose.estimatedPose.toPose2d().getTranslation());
            }
        }
        avgDist /= numTags;
                // Increase std devs based on distance (less trust at longer distances)
        if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
        
        return estStdDevs;
    }

    public static void getEstimatedCameraPose(MecanumDrive drivetrain) {
        var visionEst = getCameraFieldPos();
        visionEst.ifPresent(
                est -> {
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = getEstimationStdDevs(est);

                    drivetrain.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                });
    }
}