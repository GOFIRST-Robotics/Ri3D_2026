package frc.robot.subsystems.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

public class SimpleNavX implements GyroIO {
    private SerialPort port;
    private double currentYaw = 0;
    private double lastYaw = 0;
    private double lastTimestamp = 0;
    private double yawVelocityRadPerSec = 0;

    public SimpleNavX() {
        // Typically kUSB, kUSB1, or kUSB2 for the USB ports
        // Use 115200 baud for NavX
        port = new SerialPort(115200, SerialPort.Port.kUSB);
        lastTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        double newYaw = getYaw();
        double currentTimestamp = Timer.getFPGATimestamp();
        
        // Calculate yaw velocity
        double deltaTime = currentTimestamp - lastTimestamp;
        if (deltaTime > 0) {
            double deltaYaw = newYaw - currentYaw;
            
            // Handle wrap-around (e.g., going from 359 to 1 degrees)
            if (deltaYaw > 180) {
                deltaYaw -= 360;
            } else if (deltaYaw < -180) {
                deltaYaw += 360;
            }
            
            yawVelocityRadPerSec = Math.toRadians(deltaYaw) / deltaTime;
        }
        
        // Update stored values
        currentYaw = newYaw;
        lastTimestamp = currentTimestamp;
        
        // Set outputs
        inputs.connected = true;
        inputs.yawPosition = Rotation2d.fromDegrees(-currentYaw);
        inputs.yawVelocityRadPerSec = yawVelocityRadPerSec;
    }

    public double getYaw() {
        // Read the entire buffer
        String data = port.readString();
        
        // Find the last complete packet in the buffer (to get the freshest data)
        int startIndex = data.lastIndexOf("!y");
        
        // A full '!y' packet is 34 characters (including \r\n)
        if (startIndex != -1 && data.length() >= startIndex + 32) {
            try {
                // Yaw is 7 characters long starting at index 2 of the packet
                // Example: "!y 001.46" -> sub-string is " 001.46"
                String yawStr = data.substring(startIndex + 2, startIndex + 9);
                lastYaw = Double.parseDouble(yawStr.trim());
            } catch (Exception e) {
                // Parse error or partial packet
            }
        }
        return lastYaw;
    }
}