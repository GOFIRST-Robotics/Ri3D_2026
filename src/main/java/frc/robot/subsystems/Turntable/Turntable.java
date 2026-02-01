package frc.robot.subsystems.Turntable;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionIOPhoton;

public class Turntable extends SubsystemBase {
    private final TurntableIO io;
    private final TurntableIOInputsAutoLogged inputs = new TurntableIOInputsAutoLogged();
    private final frc.robot.subsystems.drive.MecanumDrive.MecanumDrive drive;

    public Turntable(TurntableIO io, frc.robot.subsystems.drive.MecanumDrive.MecanumDrive drive) {
        this.io = io;
        this.drive = drive;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // io.periodic();

        inputs.targetTurntableRadians = currentTargetRadians;

        Logger.processInputs("Turntable", inputs);
    }

    private double currentTargetRadians;
    public void setTargetRadians(double radians) 
    { 
        if(Math.abs(currentTargetRadians-radians)>.02){
            currentTargetRadians += Math.signum(radians-currentTargetRadians) * .02;
        } else {
            currentTargetRadians = radians;
        }
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
        if(vision.hasTarget()){
            Pose3d aprilTagPose = Constants.AprilTagFieldConstants.TAGS.get(aprilTag-1).pose;

            Pose3d robotPose = vision.getEstimatedPose3d();
            System.out.println("robot angle" + robotPose.getRotation().getZ());

            Transform3d robotToCamera = getDynamicCameraTransform();
            Transform3d robotToTurret = Constants.TurretConstants.ROBOT_TO_TURRET;
            Pose3d cameraPose = robotPose.transformBy(robotToCamera);
            Pose3d turretPose = robotPose.transformBy(robotToTurret);
            // System.out.println("Camera Pose: " + cameraPose);
            System.out.println("camera angle" + cameraPose.getRotation().getZ());
            double dx = aprilTagPose.getX() - turretPose.getX();
            double dy = aprilTagPose.getY() - turretPose.getY();

            double angleToTag = Math.atan2(dy, dx);
            System.out.println("angletotag: " + angleToTag);

            // double robotYaw = robotPose.getRotation().toRotation2d().getRadians();
            // System.out.println("robotyaw: "+robotYaw);

            double angleToTagRobot = angleToTag - cameraPose.getRotation().getZ();

            // double facingTargetRadians = Math.atan2(aprilTagPose.getY() - cameraPose.getY(), aprilTagPose.getX() - cameraPose.getX());

            angleToTagRobot = MathUtil.angleModulus(angleToTagRobot);

            // double facingTargetRadians = -Math.atan2(aprilTagPose.getY() - robotPose.getY(), aprilTagPose.getX() - robotPose.getX());
            System.out.println("robot x: " + robotPose.getX() + "robot y: " + robotPose.getY() + "tag x: " + aprilTagPose.getX() + "tag y: " + aprilTagPose.getY() + "target rads" + angleToTagRobot);
            setTargetRadians(angleToTagRobot);
        }
    }

    public Command faceAprilTagCommand(Vision vision) { return this.run(() -> faceAprilTag(vision)); }

    public Transform3d getDynamicCameraTransform() {
        Transform3d turretRotation = new Transform3d(
            new Translation3d(),
            new Rotation3d(0, 0, currentTargetRadians)
        );

        return Constants.TurretConstants.ROBOT_TO_TURRET.plus(turretRotation).plus(Constants.TurretConstants.TURRET_TO_CAMERA);
    }

}