package frc.robot.util;

public class GeometryHelper {
    public static double[] Rotate(double x, double y, double radians)
    {
        double cosTheta = Math.cos(radians);
        double sinTheta = Math.sin(radians);

        return new double[] { x * cosTheta - y * sinTheta, x * sinTheta + y * cosTheta };
    }
}
