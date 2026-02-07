package frc.robot.subsystems.Turret;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Flywheel.Flywheel;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Turntable.Turntable;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.drive.MecanumDrive.MecanumDrive;
import frc.robot.util.GeometryHelper;

public class Turret extends SubsystemBase {

    private final Flywheel flywheel;
    private final Hood hood;
    private final Turntable turntable;
    private final Field2d field = new Field2d();

    Translation3d targetPointOffset = new Translation3d(0, 0, 0);

    public Turret(Flywheel flywheel, Hood hood, Turntable turntable) {
        this.flywheel = flywheel;
        this.hood = hood;
        this.turntable = turntable;
        SmartDashboard.putData("Field", field);
    }


    @Override
    public void periodic() {
    }

    public void autoAimTurret(Translation3d point, double z_offset, Vision vision, MecanumDrive drive)
    {
        if(!vision.hasTarget()){
            return;
        }
        Pose3d robotPose = vision.getAverageEstimatedPose3d();

        SmartDashboard.putNumber("Vision Robot Angle", robotPose.getRotation().getZ());

        Transform3d robotToTurret = Constants.TurretConstants.ROBOT_TO_TURRET;
        Pose3d turretPose = robotPose.transformBy(robotToTurret);
        // Rotation3d invert = new Rotation3d(Math.PI,0,0);

        // turretPose.rotateBy(invert);

        double dx = point.getX() - turretPose.getX();
        double dy = point.getY() - turretPose.getY();

        SmartDashboard.putNumber("Target X", point.getX());
        SmartDashboard.putNumber("Target Y", point.getY());

        SmartDashboard.putNumber("Turret X", turretPose.getX());
        SmartDashboard.putNumber("Turret Y", turretPose.getY());


        double angleToTag = Math.atan2(dy, dx);

        SmartDashboard.putNumber("Angle to Tag", angleToTag);

        double angleToTagRobot = MathUtil.angleModulus(angleToTag - (drive.getGyroYaw().getRadians()+Math.PI));

        SmartDashboard.putNumber("Angle to Tag Robot", angleToTagRobot);

        // System.out.println("robot x: " + robotPose.getX() + "robot y: " + robotPose.getY() + "tag x: " + point.getX() + "tag y: " + point.getY() + "target rads" + angleToTagRobot);
        turntable.setTargetRadians(angleToTagRobot);

        double apex_height = point.getZ() + z_offset;

        double velocity_initial_y = Math.sqrt(-2 * TurretConstants.GRAVITY_CONSTANT * apex_height);

        double time_up = Math.sqrt(-2 * apex_height / TurretConstants.GRAVITY_CONSTANT);
        double time_down = Math.sqrt(-2 * apex_height / TurretConstants.GRAVITY_CONSTANT);
        double total_time = time_up+time_down;

        double horizontal_distance = Math.sqrt(dx * dx + dy * dy);
        
        double velocity_inital_x = horizontal_distance / total_time;

        double launch_angle = Math.PI/2 -Math.atan2(velocity_initial_y, velocity_inital_x);
        double launch_velocity = Math.sqrt(velocity_inital_x * velocity_inital_x + velocity_initial_y * velocity_initial_y);

        SmartDashboard.putNumber("x velo autoaim", velocity_inital_x);
        SmartDashboard.putNumber("y velo autoaim", velocity_initial_y);


        hood.setDesiredLaunchAngle(launch_angle);
        flywheel.setLaunchSpeed(launch_velocity);
        SmartDashboard.putNumber("launch angle", launch_angle);
        SmartDashboard.putNumber("launch velo", launch_velocity);


        // field.setRobotPose(vision.getOdomotreyPose2d()); 
        field.setRobotPose(vision.getRobotPose3dMerged());
        // field.setRobotPose(turretPose.toPose2d());
        // field.setRobotPose();
        // field.getObject("Turret").setPose(turretPose.toPose2d());


    }

    public boolean TurretReadyToShoot()
    {
        return flywheel.FlywheelSpeedWithinError() 
            && hood.HoodRotationWithinError()
            && turntable.TurntableHeadingWithinError();
    }

    public BooleanSupplier SubsystemsWithinError = () -> TurretReadyToShoot();

    public Command aimAndShoot(Translation3d targetPoint, double goalZOffest, Vision vision, Indexer indexer, MecanumDrive drive) {
        return this.run(() -> {
            System.out.println("wotking");
            this.autoAimTurret(targetPoint.plus(targetPointOffset), goalZOffest, vision, drive);

            if (this.TurretReadyToShoot()) {
                indexer.runIndexerDuty(0.25);
            } else {
                indexer.runIndexerDuty(0);
            }
        });
    }

    public Command offsetTargetPoint(double x, double y)
    {
        return this.run(() -> {
            targetPointOffset = targetPointOffset.plus(new Translation3d(x, y, 0));
        });
    }

    public Command resetTargetPointOffset()
    {
        return this.run(() -> {
            targetPointOffset = new Translation3d(0, 0, 0);
        });
    }
}