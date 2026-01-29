package frc.robot.subsystems.Turntable;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.subsystems.Vision.VisionIOPhoton;

import org.littletonrobotics.junction.Logger;

public class Turntable extends SubsystemBase {
    private final TurntableIO io;
    private final TurntableIOInputsAutoLogged inputs = new TurntableIOInputsAutoLogged();

    public Turntable(TurntableIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.periodic();

        inputs.targetTurntableRadians = currentTargetRadians;

        Logger.processInputs("Turntable", inputs);
    }

    private double currentTargetRadians;
    public void setTargetRadians(double radians) 
    { 
        currentTargetRadians = radians;
        if (currentTargetRadians < -Constants.TurretConstants.TURRET_TURNTABLE_MAX_RADIANS) { currentTargetRadians = -Constants.TurretConstants.TURRET_TURNTABLE_MAX_RADIANS; }
        else if (currentTargetRadians > Constants.TurretConstants.TURRET_TURNTABLE_MAX_RADIANS) { currentTargetRadians = Constants.TurretConstants.TURRET_TURNTABLE_MAX_RADIANS; }

        io.setTurntableRadians(currentTargetRadians); 
    }

    public Command incrementTurntableAngleCommand() { return this.run(() -> setTargetRadians(currentTargetRadians + Constants.TurretConstants.TURRET_TURNTABLE_CHANGE_SPEED)); }
    public Command decrementTurntableAngleCommand() { return this.run(() -> setTargetRadians(currentTargetRadians - Constants.TurretConstants.TURRET_TURNTABLE_CHANGE_SPEED)); }
    public Command setTurntableAngleCommand(double radians) { return this.run(() -> setTargetRadians(radians)); }

    public boolean TurntableHeadingWithinError()
    {
        return Math.abs(inputs.turntableRadians - currentTargetRadians) <= Constants.TurretConstants.TURRET_TURNTABLE_ACCEPTABLE_RADIAN_ERROR;
    }

    public double getYawOffsetRadiants()
    {
        return inputs.turntableRadians;
    }

    public void faceAprilTag(Vision vision)
    {
        int aprilTag = 3;
        Pose3d aprilTagPose = Constants.AprilTagFieldConstants.TAGS.get(aprilTag).pose;

        Pose3d robotPose = VisionIOPhoton.CameraToRobotPose(vision.getEstimatedPose3d(), getYawOffsetRadiants());

        double facingTargetRadians = Math.atan2(aprilTagPose.getY() - robotPose.getY(), aprilTagPose.getX() - robotPose.getX());
        setTargetRadians(facingTargetRadians);
    }

    public Command faceAprilTagCommand(Vision vision) { return this.run(() -> faceAprilTag(vision)); }
}