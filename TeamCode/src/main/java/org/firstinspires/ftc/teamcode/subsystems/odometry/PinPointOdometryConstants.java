package org.firstinspires.ftc.teamcode.subsystems.odometry;

/**
 * OdometryConstants holds all configuration constants for the OdometrySubsystem.
 * This includes encoder names, wheel geometry, and conversion factors.
 */
public class OdometryConstants {
    // Motor names for hardware mapping (update to match your configuration)
    public static final String LF_ENCODER = "leftEncoder";
    public static final String RT_ENCODER = "rightEncoder";
    public static final String BK_ENCODER = "backEncoder";

    // Odometry wheel geometry and conversion constants
    public static final double odometryCir = 31.4; // Wheel circumference in cm (example)
    public static final double odometryTick = 8192; // Encoder ticks per revolution (example)
    public static final double SideOdometryToCentre = 18.0; // Distance from side odometry to robot center (cm)
    public static final double sideOdometryAngleFromCentre = 0.0; // Angle from center (radians)
    public static final double lengthFromOdometrySideToFront = 10.0; // Distance from side odometry to front (cm)

    // Navigation system type (if used in your Specifications)
    public enum NavSystem {
        IMU,
        ODOMETRY
    }
}