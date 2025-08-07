package org.firstinspires.ftc.teamcode.subsystems.lift;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.LUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.Interval;
import org.firstinspires.ftc.teamcode.util.IntervalControl;
import org.firstinspires.ftc.teamcode.util.MotionProfileConversion;
import org.firstinspires.ftc.teamcode.util.PIDCore;
import org.firstinspires.ftc.teamcode.util.Specifications;
import org.firstinspires.ftc.teamcode.util.VelAccelPair;

/**
 * LiftSubsystem controls a multi-motor lift mechanism using PID and feedforward.
 * It supports both dual-motor and three-motor configurations.
 */
public class LiftSubsystem extends Specifications {
    // Motor controllers
    private DcMotorEx main;   // Main motor (with encoder)
    private DcMotorEx aux1;   // First auxiliary motor
    private DcMotorEx aux2;   // Second auxiliary motor (optional, for three-motor)

    // Timing control
    private ElapsedTime mainTimer;    // Timer for general timing
    private ElapsedTime safetyTimer;  // Timer for safety checks

    // PID controllers
    public PIDCore pidUp;         // PID for upward movement
    private PIDCore pidUpFF;      // PID with feedforward for up
    private PIDCore pidDown;      // PID for downward movement
    private PIDCore pidVP;        // Position-velocity PID
    private PIDCore pidVV;        // Velocity PID
    private PIDCore cascadePID;   // Cascade PID for advanced control
    private PIDCore testLiftPID = new PIDCore(1, 1, 1); // Test PID

    // Constants from LiftConstants
    private double kpUp = LiftConstants.KP_UP;
    private double kiUp = LiftConstants.KI_UP;
    private double kdUp = LiftConstants.KD_UP;
    private double kpDown = LiftConstants.KP_DOWN;
    private double kiDown = LiftConstants.KI_DOWN;
    private double kdDown = LiftConstants.KD_DOWN;
    private double kp = LiftConstants.KP;
    private double ki = LiftConstants.KI;
    private double kd = LiftConstants.KD;
    private double kpv = LiftConstants.KPV;
    private double kiv = LiftConstants.KIV;
    private double kdv = LiftConstants.KDV;
    private double kv = LiftConstants.KV;
    private double feedforward = LiftConstants.FEEDFORWARD;
    private double tickToAngleConversion = LiftConstants.TICK_TO_ANGLE;
    private double Kg = LiftConstants.DEFAULT_KG;
    private double cascadeKp = LiftConstants.CASCADE_KP;
    private double cascadeKi = LiftConstants.CASCADE_KI;
    private double cascadeKd = LiftConstants.CASCADE_KD;
    private double cascadeKpVel = LiftConstants.CASCADE_KP_VEL;
    private double cascadeKiVel = LiftConstants.CASCADE_KI_VEL;
    private double cascadeKdVel = LiftConstants.CASCADE_KD_VEL;
    private double downThreshold = LiftConstants.DOWN_THRESHOLD;
    private double m = LiftConstants.M;
    private double c = LiftConstants.C;
    private int uncertainty = LiftConstants.UNCERTAINTY;
    private final double maxVel = LiftConstants.MAX_VEL;
    private int finalPosition = LiftConstants.DEFAULT_TARGET_POS;

    // State variables
    private double power = 0;                // Current power output
    private double angularVelocity = 0;      // Angular velocity of the lift
    public boolean runToPosition = false;    // Whether to run to a target position
    private int encoderI;                    // Initial encoder position
    private int encoderF;                    // Final encoder position
    public boolean test = false;             // Test flag
    private double intervalValue = 0;        // Value from interval control
    private double cascadeOutput = 0;        // Output from cascade PID
    private double cascadeOutputPositionalValue = 0; // Positional output from cascade PID
    private double caseCadeVelocityOutputValue = 0;  // Velocity output from cascade PID
    public double powerOutput = 0;           // Output power (for telemetry)
    public double vel = 0;                   // Current velocity

    // Lookup tables for feedforward or calibration
    private InterpLUT interpLUT = new InterpLUT();
    private LUT<Integer, Double> lut = new LUT<>();

    /**
     * Enum for specifying the lift type (dual or three motor).
     */
    public enum MultiMotorType {
        dualMotor,
        threeMotor
    }

    // Motion profile conversion utility (for smooth motion)
    MotionProfileConversion motionProfileConversion;

    /**
     * Constructor for LiftSubsystem.
     * @param hardwareMap HardwareMap from OpMode
     * @param reset Whether to reset encoders on init
     * @param type MultiMotorType (dualMotor or threeMotor)
     */
    public LiftSubsystem(HardwareMap hardwareMap, boolean reset, MultiMotorType type) {
        if(type == MultiMotorType.threeMotor) {
            // Initialize all three motors for three-motor configuration
            main = hardwareMap.get(DcMotorEx.class, EXTENSION_MOTOR_MAIN);
            aux1 = hardwareMap.get(DcMotorEx.class, EXTENSION_MOTOR_AUX1);
            aux2 = hardwareMap.get(DcMotorEx.class, EXTENSION_MOTOR_AUX2);
            main.setDirection(DcMotorSimple.Direction.REVERSE);
            aux1.setDirection(DcMotorSimple.Direction.FORWARD);
            aux2.setDirection(DcMotorSimple.Direction.FORWARD);
            if (reset) {
                main.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            }
            main.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            aux1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            aux2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            main.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            aux1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            aux2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motionProfileConversion = new MotionProfileConversion(0.5,0.5,0.5);
            pidUp = new PIDCore(kpUp, kdUp, kiUp, 0, 0, Kg);
            pidDown = new PIDCore(kpDown, kdDown, kiDown);
            pidVP = new PIDCore(kp, kd, ki);
            pidVV = new PIDCore(kpv, kdv, kiv);
            mainTimer = new ElapsedTime();
            safetyTimer = new ElapsedTime();
            safetyTimer.reset();
            encoderI = getPosition();
            if (reset) {
                main.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
        else if(type == MultiMotorType.dualMotor){
            // Initialize two motors for dual-motor configuration
            main = hardwareMap.get(DcMotorEx.class, EXTENSION_MOTOR_MAIN);
            aux1 = hardwareMap.get(DcMotorEx.class, EXTENSION_MOTOR_AUX1);

            main.setDirection(DcMotorSimple.Direction.REVERSE);
            aux1.setDirection(DcMotorSimple.Direction.REVERSE);

            if (reset) {
                main.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                aux1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            main.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            aux1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            main.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            aux1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            pidUp = new PIDCore(kpUp, kdUp, kiUp, 0, 0, Kg);
            pidUpFF = new PIDCore(kp, kpUp, kiUp, feedforward);
            pidDown = new PIDCore(kpDown, kdDown, kiDown);
            pidVP = new PIDCore(kp, kd, ki);
            pidVV = new PIDCore(kpv, kdv, kiv);
            cascadePID = new PIDCore(cascadeKp, cascadeKd, cascadeKi, cascadeKpVel, cascadeKdVel, cascadeKiVel, 0);

            mainTimer = new ElapsedTime();
            encoderI = getPosition();

            lut.add(0, 0.0); // Example LUT entry
        }
    }

    // --- PID and Feedforward setters for tuning ---

    /**
     * Set position-velocity PID constants.
     */
    public void setVPPIDConstants(double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        pidVP.setConstant(kp, kd, ki);
    }

    /**
     * Set velocity PID constants.
     */
    public void setVVPIDConstants(double kp, double ki, double kd){
        this.kpv = kp;
        this.kiv = ki;
        this.kdv = kd;
        pidVV.setConstant(kpv, kdv, kiv);
    }

    /**
     * Set up/down PID constants.
     */
    public void setPidConstants(double kpUp, double kiUp, double kdUp, double kpDown, double kiDown, double kdDown){
        this.kpUp = kpUp;
        this.kiUp = kiUp;
        this.kdUp = kdUp;
        this.kpDown = kpDown;
        this.kiDown = kiDown;
        this.kdDown = kdDown;
        pidUp.setConstant(kpUp, kdUp, kiUp);
        pidDown.setConstant(kpDown, kdDown, kiDown);
    }

    /**
     * Set up/down PID constants and velocity feedforward.
     */
    public void setPidConstants(double kpUp, double kiUp, double kdUp, double kpDown, double kiDown, double kdDown, double kv){
        this.kpUp = kpUp;
        this.kiUp = kiUp;
        this.kdUp = kdUp;
        this.kpDown = kpDown;
        this.kiDown = kiDown;
        this.kdDown = kdDown;
        this.kv = kv;
        pidUp.setConstant(kpUp, kdUp, kiUp);
        pidDown.setConstant(kpDown, kdDown, kiDown);
    }

    /**
     * Set feedforward constants for PID.
     */
    public void setFeedForwardConstants(double Ka, double Kv, double Kg){
        pidUp = new PIDCore(kpUp,kdUp, kiUp, Ka, Kv, Kg);
    }

    /**
     * Set motion profile constants.
     */
    public void setMotionProfileConstants(double Ka, double Kv, double Kg){
        motionProfileConversion = new MotionProfileConversion(Ka, Kv, Kg);
    }

    // --- Motor control methods ---

    public double power2 = 0;

    /**
     * Move the lift with the specified power.
     * For dual-motor, aux1 is set to negative of main power.
     * For three-motor, all motors get the same power.
     */
    public void moveLift(double power){
        if(aux2 == null) { // dual motor
            main.setPower(power);
            aux1.setPower(-getMainPower());
            testLiftPID.outputPositional(LiftConstants.DEFAULT_TARGET_POS, getPosition());
        }
        else{
            power2 = power;
            main.setPower(power);
            aux1.setPower(power);
            aux2.setPower(power);
        }
    }

    /**
     * Move only the main lift motor.
     */
    public void moveLiftMain(double power){
        main.setPower(power);
        testLiftPID.outputPositional(LiftConstants.DEFAULT_TARGET_POS, getPosition());
    }

    /**
     * Move only the aux2 motor (if present).
     */
    public void moveLiftAux2(double power){
        if(aux2 != null) {
            aux2.setPower(power);
        }
    }

    /**
     * Move only the aux1 motor (if aux2 is present).
     */
    public void moveLiftAux(double power){
        if(aux2 != null) {
            aux1.setPower(power);
        }
    }

    /**
     * Manual control of the lift (both main and aux1).
     */
    public void manualLift(double power){
        main.setPower(power);
        aux1.setPower(power);
        testLiftPID.outputPositional(LiftConstants.DEFAULT_TARGET_POS, getPosition());
    }

    // --- Lift process methods ---

    public boolean firstReset = true;
    ElapsedTime resetTimer = new ElapsedTime();

    /**
     * Move lift to a target position using PID, with optional slow down.
     */
    public void LiftPositionalProcess(double targetPos, boolean slowDown){
        runToPosition = true;
        double pidOutput = pidUp.outputPositionalSignSwap(targetPos, getPosition()) + Kg;

        // Special case for bottoming out
        if (Math.abs(getPosition()) > 40 && Math.abs(getPosition()) < 200 && targetPos == -10){
            power = -0.9;
        } else {
            power = pidOutput;
        }

        if (slowDown){
            power *= 0.25;
        }
        moveLift(power);
    }

    /**
     * Move lift to a target position very slowly.
     */
    public void LiftPositionalProcessReallySlow(double targetPos, boolean slowDown){
        runToPosition = true;
        double pidOutput = pidUp.outputPositionalSignSwap(targetPos, getPosition()) + Kg;
        power = pidOutput;

        if (slowDown){
            power *= 0.09;
        }
        moveLift(power);
    }

    /**
     * Get the current motion profile output (velocity/acceleration pair).
     */
    public VelAccelPair getMotionProfileOutput(){
        return motionProfilePower;
    }

    VelAccelPair motionProfilePower;

    /**
     * Move lift using feedforward and motion profile.
     */
    public void LiftFeedforwardProcess(double targetPos, MotionProfileConversion motionProfileConversion){
        runToPosition = true;
        power = pidUp.outputFeedForward(targetPos, getPosition(), motionProfileConversion.getTrapezoidalConversion(getPosition(), targetPos));
        moveLift(power);
    }

    /**
     * Get Ka (acceleration feedforward constant).
     */
    public double getKa(){
        return 0; // Not stored in this class
    }

    /**
     * Get Kv (velocity feedforward constant).
     */
    public double getKv(){
        return kv;
    }

    /**
     * Get gravity compensation constant.
     */
    public double Kg(){
        return Kg;
    }

    /**
     * Set the current power value (does not move motors).
     */
    public void setPower(double power){
        this.power = power;
    }

    /**
     * Cascade PID process for advanced control.
     */
    public void LiftCascadeProcess(double targetPos, Interval... interval){
        runToPosition = true;
        IntervalControl velocityInterval = new IntervalControl(interval);
        intervalValue = velocityInterval.getOutput(getPosition());
        encoderF = getPosition();

        cascadeOutput = cascadePID.cascadeOutput(
                targetPos,
                getPosition(),
                intervalValue,
                getDerivativeValue()
        );
        moveLift(cascadeOutput);
    }

    /**
     * Simple positional feedforward control.
     */
    public void positionalFeedForward(double targetPos){
        this.feedforward = 0.00019*(getPosition());
        power = pidUp.outputPositionalSignSwap(targetPos, getPosition());
        moveLift(Math.max(power, feedforward));
    }

    /**
     * Get the current feedforward value.
     */
    public double getFeedforward(){
        return this.feedforward;
    }

    /**
     * Test cascade process for tuning.
     */
    public void testLiftCascadeProcess(double targetPos, double targetVelocity){
        runToPosition = true;
        intervalValue = targetVelocity;
        cascadeOutput = cascadePID.cascadeOutput(targetPos, getPosition(), intervalValue, getDerivativeValue());
    }

    /**
     * Start running the lift to a target position.
     */
    public void motorTurn(boolean run, int position){
        if (run){
            main.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.finalPosition = position;
            runToPosition = true;
        }
    }

    /**
     * Run the lift at a fixed power.
     */
    public void motorTurnPower(boolean run, double power){
        if (run){
            main.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            runToPosition = false;
            this.power = power + 0.2/1050*getPosition();
        }
    }

    /**
     * Stop the lift.
     */
    public void stop(boolean run){
        if (run){
            main.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            runToPosition = false;
            power = 0;
        }
    }

    /**
     * Reset the main motor encoder.
     */
    public void reset(){
        main.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        main.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Check if any motor is busy (moving to position).
     */
    public boolean isBusy() {
        return (main != null && main.isBusy()) ||
               (aux1 != null && aux1.isBusy()) ||
               (aux2 != null && aux2.isBusy());
    }

    /**
     * Get the main motor's encoder position.
     */
    public int getPosition() {
        return main.getCurrentPosition();
    }

    /**
     * Get the aux1 motor's encoder position.
     */
    public int getAuxPos(){
        return aux1.getCurrentPosition();
    }

    /**
     * Get the last set power value.
     */
    public double getPower(){
        return this.power;
    }

    /**
     * Get the main motor's actual power.
     */
    public double getMainPower(){
        return main.getPower();
    }

    /**
     * Get the aux1 motor's actual power.
     */
    public double getAux1Power(){
        return aux1.getPower();
    }

    /**
     * Get the aux2 motor's actual power (0 if not present).
     */
    public double getAux2Power(){
        return aux2 != null ? aux2.getPower() : 0;
    }

    /**
     * Get the current angular velocity (if tracked).
     */
    public double getAngularVelocity(){
        return angularVelocity;
    }

    /**
     * Check if the lift has reached its target position.
     */
    public boolean isPositionReached(){
        return Math.abs(finalPosition - getPosition()) < uncertainty;
    }

    /**
     * Get the derivative value from the test PID.
     */
    public double getDerivativeValue(){
        return -testLiftPID.getDerivative();
    }

    /**
     * Get the last error from the test PID.
     */
    public double getLastErrorValue(){
        return testLiftPID.getLastError();
    }

    /**
     * Get the current error from the test PID.
     */
    public double getErrorValue(){
        return testLiftPID.getError();
    }

    /**
     * Get the current drawn by the main motor (amps).
     */
    public double getMainCurrent(){
        return main.getCurrent(CurrentUnit.AMPS);
    }

    /**
     * Get the velocity feedforward term.
     */
    public double getKvTerm(){
        return kv * getAngularVelocity() * getPosition() / 300;
    }

    /**
     * Get the integral sum from the up PID.
     */
    public double getUpIntegralSum(){
        return pidUp.getIntegralSum();
    }

    /**
     * Get the integral sum from the down PID.
     */
    public double getDownIntegralSum(){
        return pidDown.getIntegralSum();
    }

    /**
     * Get the last target position.
     */
    public int getFinalPosition() {
        return finalPosition;
    }

    /**
     * Get the last interval value.
     */
    public double getIntervalValue() {
        return intervalValue;
    }

    /**
     * Get the last cascade PID output.
     */
    public double getCascadeOutput(){
        return cascadeOutput;
    }

    /**
     * Get the positional output from the cascade PID.
     */
    public double getCascadePositional(){
        return cascadePID.getOutputPositionalValue();
    }

    /**
     * Get the velocity output from the cascade PID.
     */
    public double getCascadeVelocity(){
        return cascadePID.getOutputVelocityValue();
    }

    /**
     * Get the derivative from the cascade PID.
     */
    public double getCascadeDerivative(){
        return cascadePID.getDerivative();
    }

    /**
     * Get the velocity derivative from the cascade PID.
     */
    public double getCascadeVelDerivative(){
        return cascadePID.getVelocityDerivative();
    }

    /**
     * Get the up PID controller instance.
     */
    public PIDCore getPidUp(){
        return pidUp;
    }
}