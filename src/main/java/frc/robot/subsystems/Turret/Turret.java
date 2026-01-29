package frc.robot.subsystems.Turret;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Flywheel.Flywheel;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Turntable.Turntable;
import frc.robot.util.GeometryHelper;

public class Turret extends SubsystemBase {

    private final Flywheel flywheel;
    private final Hood hood;
    private final Turntable turntable;

    public Turret(Flywheel flywheel, Hood hood, Turntable turntable) {
        this.flywheel = flywheel;
        this.hood = hood;
        this.turntable = turntable;
    }

    @Override
    public void periodic() {

    }

    public void autoAimTurret(double robotFieldX, double robotFieldY, double robotFieldRadians)
    {
        double[] turretWorldRelativeToRobot =
        GeometryHelper.Rotate(TurretConstants.TURRET_LOCAL_POS_X, TurretConstants.TURRET_LOCAL_POS_Y, robotFieldRadians);

        double turretWorldX = turretWorldRelativeToRobot[0] + robotFieldX;
        double turretWorldY = turretWorldRelativeToRobot[1] + robotFieldY;

        double dx = TurretConstants.GOAL_FIELD_SPACE_X_POSITION - turretWorldX;
        double dy = TurretConstants.GOAL_FIELD_SPACE_Y_POSITION - turretWorldY;

        double angleToFaceGoalField = Math.atan2(dy, dx);

        double angleToFaceGoalLocal = angleToFaceGoalField - robotFieldRadians;

        turntable.setTargetRadians(angleToFaceGoalLocal);

        double velocity_initial_y = Math.sqrt(-2 * TurretConstants.GRAVITY_CONSTANT * (TurretConstants.TURRET_VERTICAL_DISTANCE_TO_GOAL + TurretConstants.TURRET_VERTICAL_DISTANCE_APEX_OFFSET));
        double time_until_apex = -velocity_initial_y / TurretConstants.GRAVITY_CONSTANT;
        double delta_x = Math.sqrt(dx * dx + dy * dy);
        
        delta_x -= (TurretConstants.TURRET_TIME_INTO_GOAL_AFTER_APEX / (time_until_apex + TurretConstants.TURRET_TIME_INTO_GOAL_AFTER_APEX)) * delta_x;

        double velocity_inital_x = delta_x / time_until_apex;

        double launch_angle = Math.atan2(velocity_initial_y, velocity_inital_x);
        double launch_velocity = Math.sqrt(velocity_inital_x * velocity_inital_x + velocity_initial_y * velocity_initial_y);

        hood.setDesiredLaunchAngle(launch_angle);
        flywheel.setLaunchSpeed(launch_velocity);
    }

    // ...existing code...
    public void autoAimTurret2(double robotFieldX, double robotFieldY, double robotFieldRadians)
    {
        double[] turretWorldRelativeToRobot =
        GeometryHelper.Rotate(TurretConstants.TURRET_LOCAL_POS_X, TurretConstants.TURRET_LOCAL_POS_Y, robotFieldRadians);

        double turretWorldX = turretWorldRelativeToRobot[0] + robotFieldX;
        double turretWorldY = turretWorldRelativeToRobot[1] + robotFieldY;

        double dx = TurretConstants.GOAL_FIELD_SPACE_X_POSITION - turretWorldX;
        double dy = TurretConstants.GOAL_FIELD_SPACE_Y_POSITION - turretWorldY;

        double angleToFaceGoalField = Math.atan2(dy, dx);

        double angleToFaceGoalLocal = angleToFaceGoalField - robotFieldRadians;

        // 1. Wrap angle to [-PI, PI]. This handles the "no slip ring" logic physically
        // by forcing the turret to unwind if the robot spins 360 degrees.
        angleToFaceGoalLocal = Math.atan2(Math.sin(angleToFaceGoalLocal), Math.cos(angleToFaceGoalLocal));

        // 2. Clamp to physical limits (e.g. +/- 175 degrees).
        // If the target is in the "dead zone" (back of robot), aiming at the limit
        // is the safest option. The TurretReadyToShoot check will prevent firing
        // because the error will be too high.
        if (angleToFaceGoalLocal > TurretConstants.TURRET_TURNTABLE_MAX_RADIANS) {
            angleToFaceGoalLocal = TurretConstants.TURRET_TURNTABLE_MAX_RADIANS;
        } else if (angleToFaceGoalLocal < -TurretConstants.TURRET_TURNTABLE_MAX_RADIANS) {
            angleToFaceGoalLocal = -TurretConstants.TURRET_TURNTABLE_MAX_RADIANS;
        }

        turntable.setTargetRadians(angleToFaceGoalLocal);

        // Physics Calculation (using positive gravity magnitude for clarity)
        double g = Math.abs(TurretConstants.GRAVITY_CONSTANT);
        
        double velocity_initial_y = Math.sqrt(2 * g * (TurretConstants.TURRET_VERTICAL_DISTANCE_TO_GOAL + TurretConstants.TURRET_VERTICAL_DISTANCE_APEX_OFFSET));
        double time_until_apex = velocity_initial_y / g;
        double delta_x = Math.sqrt(dx * dx + dy * dy);
        
        double total_time = time_until_apex + TurretConstants.TURRET_TIME_INTO_GOAL_AFTER_APEX;

        double velocity_inital_x = delta_x / total_time;

        double launch_angle = Math.atan2(velocity_initial_y, velocity_inital_x);
        double launch_velocity = Math.sqrt(velocity_inital_x * velocity_inital_x + velocity_initial_y * velocity_initial_y);

        hood.setDesiredLaunchAngle(launch_angle);
        flywheel.setLaunchSpeed(launch_velocity);
    }

    // ...existing code...

    public boolean TurretReadyToShoot()
    {
        return flywheel.FlywheelSpeedWithinError() 
            && hood.HoodRotationWithinError()
            && turntable.TurntableHeadingWithinError();
    }

    public BooleanSupplier SubsystemsWithinError = () -> TurretReadyToShoot();
}