package frc.robot.subsystems.Vision;

import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.MecanumDrive.MecanumDrive;

public class Vision extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private final MecanumDrive drivetrain;

  private List<Pair<Pose3d, Double>> posesToAverage = new LinkedList<>();

  private final Alert disconnectedAlert =
      new Alert("Vision camera disconnected!", AlertType.kWarning);

  public Vision(VisionIO io, MecanumDrive drivetrain) {
    this.io = io;
    this.drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);

    disconnectedAlert.set(!inputs.connected);

    // Update drivetrain with vision measurement if we have a target
    if (inputs.hasTarget && inputs.connected) {
      var stdDevs = getEstimationStdDevs();
      drivetrain.addVisionMeasurement(
          inputs.estimatedPose.toPose2d(),
          inputs.timestampSeconds,
          stdDevs);
    }

    removeOldPoses(inputs.timestampSeconds);
    if (inputs.hasTarget)
    {
      removeFarPoses(inputs.estimatedPose);
      posesToAverage.add(new Pair<>(inputs.estimatedPose, inputs.timestampSeconds));
    }
    inputs.averagedEstimatedPose = getAveragedPose();

    // Log additional outputs
    Logger.recordOutput("Vision/EstimatedPose", inputs.estimatedPose.toPose2d());
    //Logger.recordOutput("Vision/AveragedEstimatedPose", inputs.averagedEstimatedPose.toPose2d());

    SmartDashboard.putString("Averaged Vision", inputs.averagedEstimatedPose.toPose2d().toString());
  }

  /**
   * Calculate standard deviations based on the estimated pose
   */
  private Matrix<N3, N1> getEstimationStdDevs() {
    var estStdDevs = Constants.SINGLE_TAG_STD_DEVS;
    
    // If we see multiple tags, use tighter standard deviations
    if (inputs.tagCount > 1) {
      estStdDevs = Constants.MULTI_TAG_STD_DEVS;
    }
    
    // Increase std devs based on distance (less trust at longer distances)
    if (inputs.tagCount == 1 && inputs.avgTagDistance > 4) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      estStdDevs = estStdDevs.times(1 + (inputs.avgTagDistance * inputs.avgTagDistance / 30));
    }
    
    return estStdDevs;
  }

  private void removeOldPoses(Double currentTimeSeconds)
  {
    posesToAverage.removeIf(pose -> (currentTimeSeconds - pose.getSecond()) > Constants.MAX_AVERAGE_TIMESTAMP_AGE);
  }

  private void removeFarPoses(Pose3d currentPose)
  {
    posesToAverage.removeIf(pose -> pose.getFirst().getTranslation().getDistance(currentPose.getTranslation()) > Constants.MAX_AVERAGE_DISTANCE_FROM_NEW_READING);
  }

  private Pose3d getAveragedPose()
  {
    if(posesToAverage.size() == 0) {
      return inputs.estimatedPose;
    }

    double xPos = 0;
    double yPos = 0;
    double zPos = 0;

    double xRot = 0;
    double yRot = 0;
    double zRot = 0;

    for(Pair<Pose3d, Double> pose : posesToAverage)
    {
      Translation3d translation = pose.getFirst().getTranslation();
      Rotation3d rotation = pose.getFirst().getRotation();

      xPos += translation.getX();
      yPos += translation.getY();
      zPos += translation.getZ();

      xRot += rotation.getX();
      yRot += rotation.getY();
      zRot += rotation.getZ();
    }

    int size = posesToAverage.size();
    return new Pose3d(xPos/size, yPos/size, zPos/size, new Rotation3d(xRot/size, yRot/size, zRot/size));
  }

  public boolean hasTarget() {
    return inputs.hasTarget;
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public Pose2d getEstimatedPose() {
    return inputs.estimatedPose.toPose2d();
  }

  public Pose3d getEstimatedPose3d()
  {
    return inputs.estimatedPose;
  }

  public Pose2d getRobotPose3dMerged()
  {
    return drivetrain.getOdometryWithVisionPose2d();
  }

  public Pose2d getOdomotreyPose2d() {
    return drivetrain.getPose();
  }

  
}