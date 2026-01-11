// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage

package frc.robot.subsystems.Gyro;


/** IO implementation for NavX (simplified - no high-frequency odometry thread). */
public class GyroIONavX implements GyroIO {
  // private final AHRS navX;

  public GyroIONavX() {
    // NavX Mini connected via USB (use kUSB1 for first USB port)
    // Change to kMXP_SPI if using the MXP port instead
    // navX = new AHRS(NavXComType.kUSB1);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // inputs.connected = navX.isConnected();
    // inputs.yawPosition = Rotation2d.fromDegrees(-navX.getAngle());
    // inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());
    // REMOVED: odometry queue processing
  }
}