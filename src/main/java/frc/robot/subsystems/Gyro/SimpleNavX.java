package frc.robot.subsystems.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

public class SimpleNavX implements GyroIO {
    private SerialPort port;
    private boolean portInitialized = false;
    
    private double currentYaw = 0;
    private double lastYaw = 0;
    private double yawOffset = 0;
    private double lastTimestamp = 0;
    private double lastValidReadTime = 0;
    private double yawVelocityRadPerSec = 0;
    
    private static final double CONNECTION_TIMEOUT = 0.5; // seconds
    private static final double VELOCITY_FILTER_CONSTANT = 0.8; // 0-1, higher = more smoothing

    public SimpleNavX() {
        // Typically kUSB, kUSB1, or kUSB2 for the USB ports
        // Use 115200 baud for NavX
        try {
            port = new SerialPort(115200, SerialPort.Port.kUSB);
            portInitialized = true;
            System.out.println("NavX serial port initialized successfully");
        } catch (Exception e) {
            System.err.println("Failed to initialize NavX serial port: " + e.getMessage());
            portInitialized = false;
        }
        lastTimestamp = Timer.getFPGATimestamp();
        lastValidReadTime = Timer.getFPGATimestamp();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        // Don't try to read if port isn't initialized
        if (!portInitialized) {
            inputs.connected = false;
            inputs.yawPosition = Rotation2d.fromDegrees(0);
            inputs.yawVelocityRadPerSec = 0;
            return;
        }
        
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
            
            // Apply low-pass filter to smooth velocity
            double rawVelocity = Math.toRadians(deltaYaw) / deltaTime;
            yawVelocityRadPerSec = (VELOCITY_FILTER_CONSTANT * yawVelocityRadPerSec) 
                                 + ((1 - VELOCITY_FILTER_CONSTANT) * rawVelocity);
        }
        
        // Update stored values
        currentYaw = newYaw;
        lastTimestamp = currentTimestamp;
        
        // Set outputs (apply yaw offset for zeroing)
        inputs.connected = isConnected();
        inputs.yawPosition = Rotation2d.fromDegrees(-(currentYaw - yawOffset));
        inputs.yawVelocityRadPerSec = yawVelocityRadPerSec;
    }

    /**
     * Zeros the yaw at the current position.
     * Call this when the robot is facing the desired "forward" direction.
     */
    public void zeroYaw() {
        yawOffset = currentYaw;
        System.out.println("NavX yaw zeroed at: " + currentYaw + " degrees");
    }

    /**
     * Sets a specific yaw offset.
     * @param offsetDegrees The offset in degrees
     */
    public void setYawOffset(double offsetDegrees) {
        yawOffset = currentYaw - offsetDegrees;
    }

    /**
     * Checks if the NavX is connected based on recent valid data.
     * @return true if data has been received within the timeout period
     */
    public boolean isConnected() {
        if (!portInitialized) {
            return false;
        }
        return (Timer.getFPGATimestamp() - lastValidReadTime) < CONNECTION_TIMEOUT;
    }

    /**
     * Gets the raw yaw value from the NavX (before offset is applied).
     * @return The raw yaw in degrees
     */
    public double getRawYaw() {
        return currentYaw;
    }

    /**
     * Gets the yaw value with offset applied.
     * @return The adjusted yaw in degrees
     */
    public double getAdjustedYaw() {
        return currentYaw - yawOffset;
    }

    private double getYaw() {
        if (!portInitialized || port == null) {
            return lastYaw;
        }
        
        try {
            // Read the entire buffer
            String data = port.readString();
            
            // Find the last complete packet in the buffer (to get the freshest data)
            int startIndex = data.lastIndexOf("!y");
            
            // A full '!y' packet is 34 characters (including \r\n)
            if (startIndex != -1 && data.length() >= startIndex + 32) {
                // Yaw is 7 characters long starting at index 2 of the packet
                // Example: "!y 001.46" -> sub-string is " 001.46"
                String yawStr = data.substring(startIndex + 2, startIndex + 9);
                lastYaw = Double.parseDouble(yawStr.trim());
                lastValidReadTime = Timer.getFPGATimestamp(); // Track successful read
            }
        } catch (Exception e) {
            // Parse error, partial packet, or serial error - keep last valid value
            System.err.println("NavX read error: " + e.getMessage());
        }
        
        return lastYaw;
    }
}