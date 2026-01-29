package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.MecanumDrive.MecanumDrive;

public class Vision extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private final MecanumDrive drivetrain;

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

    // Log additional outputs
    Logger.recordOutput("Vision/EstimatedPose", inputs.estimatedPose.toPose2d());
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

  public boolean hasTarget() {
    return inputs.hasTarget;
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public Pose2d getEstimatedPose() {
    return inputs.estimatedPose.toPose2d();
  }
}