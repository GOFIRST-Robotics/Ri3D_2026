package frc.robot.subsystems.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimpleNavXStandalone extends SubsystemBase {
    private SerialPort port;
    private boolean portInitialized = false;
    
    // Variables for continuous angle tracking (fixes offset issues)
    private double latestRawYaw = 0;       // Current reading from sensor (0-360)
    private double previousRawYaw = 0;     // Reading from the previous loop
    private double accumulatedYaw = 0;     // Continuous total angle (unbounded)

    private double yawOffset = 0;
    private double lastTimestamp = 0;
    private double lastValidReadTime = 0;
    private double yawVelocityRadPerSec = 0;
    
    private static final double CONNECTION_TIMEOUT = 0.5; // seconds
    private static final double VELOCITY_FILTER_CONSTANT = 0.8; // 0-1, higher = more smoothing

    // Buffer to hold split packets between loops
    private String serialBuffer = ""; 
    // Flag to prevent logic running before first real read
    private boolean hasValidData = false;

    public SimpleNavXStandalone() {
        try {
            port = new SerialPort(115200, SerialPort.Port.kUSB);
            portInitialized = true;
            System.out.println("NavX serial port initialized successfully");
        } catch (Exception e) {
            System.err.println("Failed to initialize NavX serial port: " + e.getMessage());
            portInitialized = false;
        }
        
        // Initialize values to prevent startup startup jumps
        updateSerialData(); 
        previousRawYaw = latestRawYaw;
        accumulatedYaw = latestRawYaw; 
        
        lastTimestamp = Timer.getFPGATimestamp();
        lastValidReadTime = Timer.getFPGATimestamp();
    }

  @Override
    public void periodic() {
        if (!portInitialized) return;
        
        // 1. Update latestRawYaw from Serial
        updateSerialData();

        // SAFETY CHECK: Don't run physics if we haven't received real data yet
        if (!hasValidData) {
            previousRawYaw = latestRawYaw; // Keep them synced so delta is 0
            accumulatedYaw = latestRawYaw; // Prevent jumping from 0 to actual angle
            return; 
        }

        double currentTimestamp = Timer.getFPGATimestamp();
        
        // 2. Calculate raw change and handle 0-360 wrap
        double deltaYaw = latestRawYaw - previousRawYaw;
        
        // Normalize delta to -180 to 180 (shortest path)
        if (deltaYaw > 180) {
            deltaYaw -= 360;
        } else if (deltaYaw < -180) {
            deltaYaw += 360;
        }

        // 3. Accumulate total continuous yaw
        accumulatedYaw += deltaYaw;

        // 4. Calculate Velocity
        double deltaTime = currentTimestamp - lastTimestamp;
        
        if (deltaTime > 0) {
            double rawVelocity = 0;
            
            // Only calc velocity if we moved enough to be outside noise floor
            if (Math.abs(deltaYaw) > 0.001) {
                rawVelocity = Math.toRadians(deltaYaw) / deltaTime;
            } 
            
            // Filter
            yawVelocityRadPerSec = (VELOCITY_FILTER_CONSTANT * yawVelocityRadPerSec) 
                                + ((1 - VELOCITY_FILTER_CONSTANT) * rawVelocity);
                                
            lastTimestamp = currentTimestamp;
        }
        
        previousRawYaw = latestRawYaw;
    }

    // ...existing code...

    // private void updateSerialData() {
    //     if (!portInitialized || port == null) return;
        
    //     try {
    //         int bytesAvailable = port.getBytesReceived();
    //         if (bytesAvailable > 0) {
    //             // Read whatever is available
    //             String incoming = port.readString();
                
    //             // DEBUG: Print what we're receiving
    //             System.out.println("Serial bytes: " + bytesAvailable + " | Raw: " + incoming);
                
    //             if (incoming == null) return;
                
    //             // Append to our buffer (handles split packets)
    //             serialBuffer += incoming;
                
    //             // DEBUG: Print buffer state
    //             System.out.println("Buffer: " + serialBuffer);
                
    //             // Safety: prevent buffer from growing infinite if parsing fails
    //             if (serialBuffer.length() > 1000) {
    //                 System.out.println("WARNING: Buffer overflow, clearing");
    //                 serialBuffer = "";
    //             }

    //             // Process ALL complete packets in the buffer
    //             while (true) {
    //                 int packetStart = serialBuffer.indexOf("!y");
    //                 int packetEnd = serialBuffer.indexOf('\r', packetStart == -1 ? 0 : packetStart);
                    
    //                 // DEBUG: Print parsing state
    //                 System.out.println("PacketStart: " + packetStart + " | PacketEnd: " + packetEnd);
                    
    //                 if (packetStart == -1) break;
    //                 if (packetEnd == -1) break;

    //                 String packet = serialBuffer.substring(packetStart, packetEnd);
    //                 System.out.println("Found packet: [" + packet + "]");
                    
    //                 try {
    //                     String valStr = packet.substring(2).trim();
    //                     latestRawYaw = Double.parseDouble(valStr);
    //                     System.out.println("Parsed yaw: " + latestRawYaw);
                        
    //                     lastValidReadTime = Timer.getFPGATimestamp();
    //                     hasValidData = true;
    //                 } catch (Exception e) {
    //                     System.out.println("Parse error: " + e.getMessage());
    //                 }

    //                 serialBuffer = serialBuffer.substring(packetEnd + 1);
    //             }
    //         }
    //     } catch (Exception e) {
    //         System.out.println("Serial error: " + e.getMessage());
    //     }
    // }

    // ...existing code...

    private void updateSerialData() {
        if (!portInitialized || port == null) return;
        
        try {
            int bytesAvailable = port.getBytesReceived();
            if (bytesAvailable > 0) {
                String incoming = port.readString();
                
                if (incoming == null) return;
                
                serialBuffer += incoming;
                
                // Safety: prevent buffer from growing infinite if parsing fails
                if (serialBuffer.length() > 1000) {
                    serialBuffer = "";
                }

                // Process ALL complete packets in the buffer
                // Look for lines ending with \r\n or just \n
                while (true) {
                    int packetStart = serialBuffer.indexOf("!y");
                    int packetEnd = serialBuffer.indexOf('\n', packetStart == -1 ? 0 : packetStart);
                    
                    if (packetStart == -1 || packetEnd == -1) break;

                    String packet = serialBuffer.substring(packetStart, packetEnd);
                    
                    try {
                        // Format: "!y 144.25-011.41 003.11 318.53CF"
                        // Skip "!y " (3 chars) to get to the yaw value
                        String afterPrefix = packet.substring(3).trim();
                        
                        // Find where yaw ends (at the next '-' or space after first char)
                        int yawEnd = -1;
                        for (int i = 1; i < afterPrefix.length(); i++) {
                            char c = afterPrefix.charAt(i);
                            if (c == '-' || c == ' ') {
                                yawEnd = i;
                                break;
                            }
                        }
                        
                        if (yawEnd > 0) {
                            String yawStr = afterPrefix.substring(0, yawEnd);
                            latestRawYaw = Double.parseDouble(yawStr);
                            
                            lastValidReadTime = Timer.getFPGATimestamp();
                            hasValidData = true;
                        }
                    } catch (Exception e) {
                        System.out.println("Parse error: " + e.getMessage());
                    }

                    serialBuffer = serialBuffer.substring(packetEnd + 1);
                }
            }
        } catch (Exception e) {
            System.out.println("Serial error: " + e.getMessage());
        }
    }

// ...existing code...

    /**
     * Gets the current yaw angle as a Rotation2d.
     * @return The yaw angle with offset applied (CCW+)
     */
    public Rotation2d getRotation2d() {
        // Negate to convert NavX CW+ to standard CCW+
        return Rotation2d.fromDegrees(-(accumulatedYaw - yawOffset));
    }

    /**
     * Gets the current yaw angle in degrees.
     * @return The yaw angle (-180 to 180) with offset applied
     */
    public double getYawDegrees() {
        return getRotation2d().getDegrees();
    }

    public double getYawRadians() {
        return getRotation2d().getRadians();
    }

    /**
     * Gets the yaw velocity in radians per second.
     * @return The filtered yaw velocity (CCW+)
     */
    public double getYawVelocityRadPerSec() {
        // Negate velocity to match CCW+ position convention
        return -yawVelocityRadPerSec;
    }

    public double getYawVelocityDegreesPerSec() {
        return Math.toDegrees(getYawVelocityRadPerSec());
    }

    /**
     * Zeros the yaw at the current position.
     */
    public void zeroYaw() {
        yawOffset = accumulatedYaw;
        System.out.println("NavX yaw zeroed at: " + accumulatedYaw);
    }

    /**
     * Sets the current yaw to a specific angle.
     * @param angleDegrees The desired current heading (CCW+)
     */
    public void setYaw(double angleDegrees) {
        // Output = -(Input - Offset)  => Offset = Input + Output
        yawOffset = accumulatedYaw + angleDegrees;
    }

    public boolean isConnected() {
        if (!portInitialized) return false;
        return (Timer.getFPGATimestamp() - lastValidReadTime) < CONNECTION_TIMEOUT;
    }

    public double getRawYaw() {
        return latestRawYaw;
    }

    
}