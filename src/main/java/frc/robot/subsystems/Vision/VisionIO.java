package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public boolean hasTarget = false;
    public Pose3d estimatedPose = new Pose3d();
    public double timestampSeconds = 0.0;
    public int tagCount = 0;
    public double avgTagDistance = 0.0;
    public int[] tagIds = new int[0];
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}
}