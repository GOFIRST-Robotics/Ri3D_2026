package frc.robot.subsystems.Flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    public Flywheel(FlywheelIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.periodic();
        Logger.processInputs("Flywheel", inputs);
    }

    double topTargetRPM;
    double bottomTargetRPM;
    public void runFlywheels(double topRPM, double bottomRPM)
    {
        topTargetRPM = topRPM;
        bottomTargetRPM = bottomRPM;

        if(topRPM+bottomRPM<1){
            io.setkDutyZero();
        } else {
            io.setTopFlywheelRPM(topRPM);
            io.setBottomFlywheelRPM(bottomRPM);
        }
    }

    public void setLaunchSpeed(double launchSpeed)
    {
        double rpm = (60 / Math.PI) * (launchSpeed / Constants.WHEEL_DIAMETER);

        runFlywheels(rpm, rpm);
        runFlywheels(rpm, rpm);
    }

    double setRPM = 3000;
    public void IncrementSetRPM(double change)
    {
        setRPM += change;
        if (setRPM < Constants.TurretConstants.TURRET_FLYWHEEL_MIN_RPM) {
            setRPM = Constants.TurretConstants.TURRET_FLYWHEEL_MIN_RPM;
        }
        if (setRPM > Constants.TurretConstants.TURRET_FLYWHEEL_MAX_RPM) {
            setRPM = Constants.TurretConstants.TURRET_FLYWHEEL_MAX_RPM;
        }

        runFlywheels((setRPM-1000.0)*3.0, setRPM);
    }

    public Command StopFlywheelsCommand() { return this.runOnce(() -> runFlywheels(0, 0)); }
    public Command decrementRpmSetPoint() { return this.run(() -> IncrementSetRPM(-Constants.TurretConstants.TURRET_FLYWHEEL_CHANGE_SPEED)); }
    public Command incrementRpmSetPoint() { return this.run(() -> IncrementSetRPM(Constants.TurretConstants.TURRET_FLYWHEEL_CHANGE_SPEED)); }

    public boolean FlywheelSpeedWithinError()
    {
        return Math.abs(inputs.bottomFlywheelRPM - bottomTargetRPM) <= Constants.TurretConstants.TURRET_FLYWHEEL_ACCEPTABLE_RPM_ERROR && Math.abs(inputs.topFlywheelRPM - topTargetRPM) <= Constants.TurretConstants.TURRET_FLYWHEEL_ACCEPTABLE_RPM_ERROR;
    }
}