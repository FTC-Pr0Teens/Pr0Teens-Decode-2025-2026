package org.firstinspires.ftc.teamcode.subsystems.lift;

public class LiftConstants {

    public enum MultiMotorType {
        dualMotor,
        threeMotor
    }

    // PID Constants
    public static final double KP_UP = 0.0055;
    public static final double KI_UP = 1.0 / 450;
    public static final double KD_UP = 0.0;

    public static final double KP_DOWN = 0.007;
    public static final double KI_DOWN = 0.04;
    public static final double KD_DOWN = 0.0;

    public static final double KP = 0.011;
    public static final double KI = 0.0;
    public static final double KD = 0.0;

    public static final double KPV = 0.02;
    public static final double KIV = 0.0;
    public static final double KDV = 0.0;

    // Feedforward & Physical Constants
    public static final double FEEDFORWARD = 0.05;
    public static final double TICK_TO_ANGLE = Math.PI * 2 / 753.2;
    public static final double DEFAULT_KG = 0.215;

    // Cascade PID Constants
    public static final double CASCADE_KP = 1.0 / 600;
    public static final double CASCADE_KI = 0.0;
    public static final double CASCADE_KD = 0.0;
    public static final double CASCADE_KP_VEL = 1.0 / 2500;
    public static final double CASCADE_KI_VEL = 0.0;
    public static final double CASCADE_KD_VEL = 0.0;

    // Limits & Thresholds
    public static final double KV = 0.3;
    public static final double DOWN_THRESHOLD = -0.3;
    public static final double M = 0.1;
    public static final double C = 0.1;
    public static final int UNCERTAINTY = 100;
    public static final double MAX_VEL = 2000;
    public static final int DEFAULT_TARGET_POS = 3900;
}
