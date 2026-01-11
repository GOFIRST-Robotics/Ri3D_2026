package frc.robot.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Flywheel.Flywheel;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Turntable.Turntable;
import frc.robot.util.GeometryHelper;

public class Turret extends SubsystemBase {

    private final Flywheel flywheel;
    private final Hood hood;
    private final Turntable turntable;

    // Dependency Injection: We pass the subsystems in, we don't create them here.
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
        double time_at_apex = -velocity_initial_y / TurretConstants.GRAVITY_CONSTANT;
        double delta_x = Math.sqrt(dx * dx + dy * dy);
        
        delta_x -= (TurretConstants.TURRET_TIME_INTO_GOAL_AFTER_APEX / (time_at_apex + TurretConstants.TURRET_TIME_INTO_GOAL_AFTER_APEX)) * delta_x;

        double velocity_inital_x = delta_x / time_at_apex;

        double launch_angle = Math.atan2(velocity_initial_y, velocity_inital_x);
        double launch_velocity = Math.sqrt(velocity_inital_x * velocity_inital_x + velocity_initial_y * velocity_initial_y);

        hood.setTargetRadians(launch_angle);
        flywheel.setLaunchSpeed(launch_velocity);
    }
}