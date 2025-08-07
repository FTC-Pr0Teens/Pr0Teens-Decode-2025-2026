package org.firstinspires.ftc.teamcode.subsystems.mecanum;

/**
 * MecanumConstants holds all configuration constants for the MecanumSubsystem.
 * This includes scaling factors, maximum velocities, and motor names.
 */
public class MecanumConstants {
    /** Power scaling factor for all drive motors (0-1) */
    public static final double SCALE = 1.0;

    /** Maximum allowed angular velocity for velocity control (radians/sec) */
    public static final double MAX_ANGULAR_VEL = 10.0; // Adjust as needed

    // Motor names for hardware mapping (update to match your configuration)
    public static final String BKLF_MOTOR = "leftBack";
    public static final String BKRT_MOTOR = "rightBack";
    public static final String FTLF_MOTOR = "leftForward";
    public static final String FTRT_MOTOR = "rightForward";
}