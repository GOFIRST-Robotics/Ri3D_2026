package frc.robot.subsystems.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A standalone NavX gyro driver that reads yaw data over serial USB.
 * Handles continuous angle tracking (beyond ±180°) and calculates angular velocity.
 */
public class SimpleNavXStandalone extends SubsystemBase {
    // Serial port connection to the NavX
    private SerialPort port;
    private boolean portInitialized = false;
    
    // Yaw tracking variables
    private double latestRawYaw = 0;       // Most recent yaw reading from NavX (-180 to 180)
    private double previousRawYaw = 0;     // Previous loop's raw yaw (for delta calculation)
    private double accumulatedYaw = 0;     // Continuous yaw that tracks beyond ±180°

    // Offset applied when zeroing or setting the gyro angle
    private double yawOffset = 0;
    
    // Timing variables for velocity calculation
    private double lastTimestamp = 0;
    private double yawVelocityRadPerSec = 0;
    
    // Low-pass filter constant for smoothing velocity (0-1, higher = more smoothing)
    private static final double VELOCITY_FILTER_CONSTANT = 0.8; 

    // Buffer for assembling serial data packets
    private String serialBuffer = ""; 
    private boolean hasValidData = false;

    /**
     * Creates a new SimpleNavXStandalone and attempts to connect via USB.
     */
    public SimpleNavXStandalone() {
        tryConnect();
        
        // Initialize yaw tracking if connection succeeded
        if (portInitialized) {
            updateSerialData(); 
            previousRawYaw = latestRawYaw;
            accumulatedYaw = latestRawYaw; 
        }
        
        lastTimestamp = Timer.getFPGATimestamp();
    }

    /**
     * Attempts to connect to the NavX on any available USB port.
     */
    private void tryConnect() {
        SerialPort.Port[] portsToTry = {
            SerialPort.Port.kUSB,
            SerialPort.Port.kUSB1,
            SerialPort.Port.kUSB2
        };
        
        for (SerialPort.Port portOption : portsToTry) {
            try {
                port = new SerialPort(115200, portOption);
                portInitialized = true;
                serialBuffer = "";
                hasValidData = false;
                System.out.println("NavX serial port initialized on " + portOption);
                return;
            } catch (Exception e) {
                // Port unavailable, try next one
            }
        }
        
        System.err.println("Failed to initialize NavX on any USB port");
        portInitialized = false;
    }

    /**
     * Called every robot loop (~20ms). Updates yaw data and calculates velocity.
     */
    @Override
    public void periodic() {
        if (!portInitialized) return;
        
        // Read new data from serial port
        updateSerialData();

        // Check if port disconnected during read
        if (!portInitialized) return;

        // Wait until we have valid data before processing
        if (!hasValidData) {
            previousRawYaw = latestRawYaw; 
            accumulatedYaw = latestRawYaw; 
            return; 
        }

        double currentTimestamp = Timer.getFPGATimestamp();
        
        // Calculate change in yaw since last loop
        double deltaYaw = latestRawYaw - previousRawYaw;
        
        // Handle wraparound when crossing ±180° boundary
        if (deltaYaw > 180) {
            deltaYaw -= 360;
        } else if (deltaYaw < -180) {
            deltaYaw += 360;
        }

        // Add delta to accumulated yaw for continuous tracking
        accumulatedYaw += deltaYaw;

        // Calculate angular velocity with low-pass filtering
        double deltaTime = currentTimestamp - lastTimestamp;
        if (deltaTime > 0) {
            double rawVelocity = 0;
            
            if (Math.abs(deltaYaw) > 0.001) {
                rawVelocity = Math.toRadians(deltaYaw) / deltaTime;
            } 
            
            // Apply exponential smoothing filter to reduce noise
            yawVelocityRadPerSec = (VELOCITY_FILTER_CONSTANT * yawVelocityRadPerSec) 
                                + ((1 - VELOCITY_FILTER_CONSTANT) * rawVelocity);
                                
            lastTimestamp = currentTimestamp;
        }
        
        previousRawYaw = latestRawYaw;
    }

    /**
     * Reads and parses serial data from the NavX.
     * Looks for packets starting with "!y" containing yaw data.
     */
    private void updateSerialData() {
        if (!portInitialized || port == null) return;
        
        try {
            int bytesAvailable = port.getBytesReceived();
            
            if (bytesAvailable > 0) {
                String incoming = port.readString();
                if (incoming == null) return;
                
                // Append new data to buffer
                serialBuffer += incoming;
                
                // Prevent buffer overflow
                if (serialBuffer.length() > 1000) {
                    serialBuffer = "";
                }

                // Process all complete packets in buffer
                while (true) {
                    int packetStart = serialBuffer.indexOf("!y");
                    int packetEnd = serialBuffer.indexOf('\n', packetStart == -1 ? 0 : packetStart);
                    
                    // Exit if no complete packet available
                    if (packetStart == -1 || packetEnd == -1) break;

                    String packet = serialBuffer.substring(packetStart, packetEnd);
                    
                    // Parse yaw value from packet
                    try {
                        String afterPrefix = packet.substring(2);
                        if (afterPrefix.startsWith(" ")) {
                            afterPrefix = afterPrefix.substring(1);
                        }
                        
                        // Find end of yaw number (next '-' or space)
                        int yawEnd = -1;
                        int startIndex = (afterPrefix.length() > 0 && afterPrefix.charAt(0) == '-') ? 1 : 0;
                        
                        for (int i = startIndex + 1; i < afterPrefix.length(); i++) {
                            char c = afterPrefix.charAt(i);
                            if (c == '-' || c == ' ') {
                                yawEnd = i;
                                break;
                            }
                        }
                        
                        if (yawEnd > 0) {
                            String yawStr = afterPrefix.substring(0, yawEnd);
                            latestRawYaw = Double.parseDouble(yawStr);
                            hasValidData = true;
                        }
                    } catch (NumberFormatException e) {
                        // Skip malformed packets
                    }

                    // Remove processed packet from buffer
                    serialBuffer = serialBuffer.substring(packetEnd + 1);
                }
            }
        } catch (Throwable e) {
            // Disable NavX on any error (e.g., unplugged) to prevent crashes
            System.err.println("NavX Error: " + e.getMessage() + ". Disabling NavX.");
            portInitialized = false;
            try {
                port.close();
            } catch (Exception ex) {
                // Ignore close errors
            }
        }
    }

    /**
     * @return The current yaw as a Rotation2d (negated for WPILib convention)
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(-(accumulatedYaw - yawOffset));
    }

    /**
     * @return The current unwrapped yaw in degrees (with offset applied)
     */
    public double getYawDegrees() {
        return getRotation2d().getDegrees();
    }

    /**
     * @return The angular velocity in degrees per second
     */
    public double getYawVelocityDegreesPerSec() {
        return Math.toDegrees(-yawVelocityRadPerSec);
    }

    /**
     * Resets the yaw to zero at the current heading.
     */
    public void zeroYaw() {
        yawOffset = accumulatedYaw;
    }

    /**
     * Sets the current heading to a specific angle.
     * @param angleDegrees The angle to set as current heading
     */
    public void setYaw(double angleDegrees) {
        yawOffset = accumulatedYaw + angleDegrees;
    }

    /**
     * @return true if the NavX is connected and communicating
     */
    public boolean isConnected() {
        return portInitialized;
    }

    /**
     * @return The raw yaw value directly from NavX (-180 to 180)
     */
    public double getRawYaw() {
        return latestRawYaw;
    }

    /**
     * Sends initialization commands to the NavX to configure yaw streaming.
     * Sends "!Sa3C4B" to set up the stream format, then "!y" to request yaw data.
     */
    public void sendInitCommands() {
        if (!portInitialized || port == null) {
            System.err.println("Cannot send commands: NavX not connected");
            return;
        }
        
        try {
            // Send stream configuration command
            port.writeString("!Sa3C4B\n");
            
            // Small delay to allow NavX to process
            Timer.delay(0.05);
            
            // Send yaw request command
            port.writeString("!y\n");
            
            System.out.println("NavX init commands sent successfully");
        } catch (Exception e) {
            System.err.println("Failed to send NavX commands: " + e.getMessage());
        }
    }

    /**
     * Sends a custom command string to the NavX.
     * @param command The command to send (newline will be appended)
     */
    public void sendCommand(String command) {
        if (!portInitialized || port == null) {
            System.err.println("Cannot send command: NavX not connected");
            return;
        }
        
        try {
            port.writeString(command + "\n");
        } catch (Exception e) {
            System.err.println("Failed to send NavX command: " + e.getMessage());
        }
    }

}