/**
 * ===============================================================================
 * FIRST TECH CHALLENGE - BLUE FAR SIDE AUTONOMOUS PROGRAM
 * ===============================================================================
 *
 * FILE: BlueFieldAuto.java
 * TEAM: Pr0Teens (FTC Team)
 * SEASON: 2025-2026
 *
 * DESCRIPTION:
 * This is a specialized Blue Alliance autonomous designed for the far starting
 * position. Unlike the standard Blue autonomous, this program emphasizes:
 *   - Minimal initial movement to scoring position
 *   - Three-specimen preload scoring sequence with custom velocity tuning
 *   - Extended intake operations with sequential sorter positioning
 *   - Optimized for far-side field geometry and scoring angles
 *
 * KEY FEATURES:
 * - Timer-based state transitions (uses elapsed time instead of stage counting)
 * - Custom shooter velocity PIDF coefficients for far-side shooting
 * - Simplified state machine with intake-focused second phase
 * - Dedicated telemetry messages for debugging transitions
 *
 * TIMING ARCHITECTURE:
 * Unlike the standard autonomous which uses stage-based control, this program
 * uses a continuous timer (pusherTimer) with millisecond thresholds to control
 * transitions within each state. This provides more precise timing for the
 * far-side shooting requirements.
 *
 * SPECIAL DEDICATION:
 * This autonomous includes telemetry dedications to Sydney Wong and "the big sw".
 * ===============================================================================
 */
package org.firstinspires.ftc.teamcode.opmodes.tests;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
//import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretSubsystem;



@Autonomous(name = "far leave ")
public class BlueFieldAuto extends LinearOpMode {
    // Subsystem command objects
    private MecanumCommand mecanumCommand;
    private TurretSubsystem turretSubsystem;
    // IntakeCommand is disabled - using direct motor control for this autonomous
    // private IntakeCommand intakeCommand;
    private OuttakeCommand outtakeCommand;

    /**
     * State machine enumeration for far-side Blue autonomous.
     * Simplified compared to standard autonomous - focuses on preload scoring
     * and basic intake operations without extended multi-cycle sequences.
     */
    enum AUTO_STATE {
        MOVEPRELOAD,
        PRELOAD_ONE,
        PRELOAD_TWO,
        PRELOAD_THREE,
        INTAKE_ONE,
        INTAKE_MOVE,
        INTAKE_TWO,
        INTAKE_THREE,
        TURN_ONE,
        SUBMERSIBLE_PICKUP,
        PICKUP_FIRST,

    }

    // Initialize to first state
    AUTO_STATE autoState = AUTO_STATE.MOVEPRELOAD;
    // Servo position constants for far-side pusher mechanism
    // Note: Different values than standard autonomous due to far-side geometry
    private static final double PUSHER_UP = 0.39;
    private static final double PUSHER_DOWN = 0.0;
    private static final double PUSHER_UP1 = 0.19;
    private static final double PUSHER_DOWN1 = 0;
    private static final long PUSHER_TIME = 300;
    // Single timer for all timed operations in this autonomous
    private final ElapsedTime pusherTimer = new ElapsedTime();

    // Pusher state tracking
    private boolean isPusherUp = false;
    // PID tuning constants for far-side Blue Alliance control
    public static double kpx = 0.058;
    public static double kpy = 0.058;
    public static double kdx = 0.0023;
    public static double kdy = 0.0023;
    public static double kpTheta = 1.3;
    public static double kdTheta = 0.0095;
    // Integral gains (currently disabled)
    public static double kix = 0;
    public static double kiy = 0;
    public static double kitheta = 40000;
    // Sorter mechanism control
    int sorterpos = 0;
    private static final double SORTER_FIRST_POS = 0.0;
    private static final double SORTER_SECOND_POS = 0.45;
    private static final double SORTER_THIRD_POS = 0.90;

    // Stage variable for timer-based state transitions
    double stage = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware singleton and subsystems
        Hardware hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        // IntakeCommand disabled - using direct motor control
        // intakeCommand = new IntakeCommand(hw);
        turretSubsystem = new TurretSubsystem(hw);
        outtakeCommand = new OuttakeCommand(hw);
        // Initialize servos to safe starting positions
        hw.pusher.setPosition(PUSHER_DOWN1);
        hw.pusher1.setPosition(PUSHER_DOWN);
        hw.sorter.setPosition(0);
        hw.sorter.setPosition(SORTER_FIRST_POS);
        // Configure shooter for velocity control
        hw.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Set motor directions for proper operation
        hw.pusher1.setDirection(Servo.Direction.REVERSE);
        hw.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        // Apply PID constants to mecanum drive system
        mecanumCommand.setConstants(kpx, kdx, kix,
                kpy, kdy, kiy,
                kpTheta, kdTheta, kitheta);
        // Initialize timing and state flags
        ElapsedTime timer = new ElapsedTime();
        boolean paused = false;
        boolean submersibleTargetSet = false;
        // Wait for driver station start signal
        waitForStart();
        // Reset pushers to down position at start
        hw.pusher.setPosition(PUSHER_DOWN1);
        hw.pusher1.setPosition(PUSHER_DOWN);
        // Main autonomous loop
        while (opModeIsActive()) {
            telemetry.addLine("for sydney wong");
            // Core subsystem processing (must run every loop)
            mecanumCommand.motorProcess();
            mecanumCommand.processPIDUsingPinpoint();
            mecanumCommand.processOdometry();
            // State machine - execute current autonomous phase
            switch (autoState) {
                case MOVEPRELOAD:
                    /**
                     * PHASE 1: Initial movement to far-side scoring position
                     * Minimal movement (40, 0) with slight rotation (0.6 rad)
                     * Includes safety delay before transitioning
                     */
                    mecanumCommand.moveToPos(40, 0, 0.6);
                    pusherTimer.reset();

                    if (mecanumCommand.isPositionReached() && pusherTimer.milliseconds() >=200) {
                        mecanumCommand.stop();
//                        autoState = AUTO_STATE.PRELOAD_ONE;

                    }
                    break;


                case PRELOAD_ONE:
                    /**
                     * PHASE 2: Score first preloaded artifact (far-side)
                     * Uses custom velocity PIDF (70, 0, 0, 0) for far shooting
                     * Timer-based sequence: 700ms delay -> push 300ms -> retract 600ms
                     */
                    hw.intake.setPower(1.0);
                    hw.shooter.setVelocityPIDFCoefficients(70, 0, 0, 0);
                    outtakeCommand.spinupfar();
                    if (stage == 0) {
                        pusherTimer.reset();
                        stage++;
                    } else if (stage == 1 && pusherTimer.milliseconds() >= 700) {
                        hw.pusher.setPosition(PUSHER_UP);
                        hw.pusher1.setPosition(PUSHER_UP1);
                        pusherTimer.reset();
                        stage++;
                    } else if (stage == 2 && pusherTimer.milliseconds() >= 300) {
                        hw.pusher.setPosition(PUSHER_DOWN);
                        hw.pusher1.setPosition(PUSHER_DOWN1);
                        pusherTimer.reset();
                        stage++;
                    } else if (stage == 3 && pusherTimer.milliseconds() >= 600) {
                        hw.sorter.setPosition(SORTER_SECOND_POS);
                        autoState = AUTO_STATE.PRELOAD_TWO;
                        pusherTimer.reset();
                        stage = 0;
                    }

                    break;


                case PRELOAD_TWO:
                    /**
                     * PHASE 3: Score second preloaded artifact
                     * Uses velocity PIDF (67, 0, 0, 0) - slightly lower than first
                     * Shorter initial delay (500ms) compared to PRELOAD_ONE
                     */
                    hw.intake.setPower(1.0);

                    hw.shooter.setVelocityPIDFCoefficients(67, 0, 0, 0);
                    outtakeCommand.spinupfar();

                    if (stage == 0 && pusherTimer.milliseconds() >= 500) {
                        hw.pusher.setPosition(PUSHER_UP);
                        hw.pusher1.setPosition(PUSHER_UP1);
                        stage++;
                        pusherTimer.reset();
                    } else if (stage == 1 && pusherTimer.milliseconds() >= 300) {
                        hw.pusher.setPosition(PUSHER_DOWN);
                        hw.pusher1.setPosition(PUSHER_DOWN1);

                        stage++;
                        pusherTimer.reset();
                    } else if (stage == 2 && pusherTimer.milliseconds() >= 600) {
                        pusherTimer.reset();
                        hw.sorter.setPosition(SORTER_THIRD_POS);
                        autoState = AUTO_STATE.PRELOAD_THREE;
                        stage = 0;
                    }


                    break;
                case PRELOAD_THREE:
                    /**
                     * PHASE 4: Score third preloaded artifact
                     * Final preload scoring before transitioning to intake operations
                     * Shooter is stopped after this phase to conserve power
                     */
                    hw.intake.setPower(1.0);
                    hw.shooter.setVelocityPIDFCoefficients(67, 0, 0, 0);
                    outtakeCommand.spinupfar();
                    if (stage == 0) {
                        pusherTimer.reset();
                        stage++;
                    } else if (stage == 1 && pusherTimer.milliseconds() >= 600) {
                        hw.pusher.setPosition(PUSHER_UP);
                        hw.pusher1.setPosition(PUSHER_UP1);
                        pusherTimer.reset();
                        stage++;
                    } else if (stage == 2 && pusherTimer.milliseconds() >= 500) {
                        hw.pusher.setPosition(PUSHER_DOWN);
                        hw.pusher1.setPosition(PUSHER_DOWN1);
                        pusherTimer.reset();
                        stage++;
                    } else if (stage == 3 && pusherTimer.milliseconds() >= 500) {
                        hw.sorter.setPosition(SORTER_SECOND_POS);
                        pusherTimer.reset();
                        outtakeCommand.stopShooter();
                        autoState = AUTO_STATE.INTAKE_MOVE;
                        stage = 0;
                    }

                    break;
                case INTAKE_MOVE:
                    /**
                     * PHASE 5: Navigate to intake position
                     * Moves to (112, -39) facing -π/2 radians
                     * Resets sorter to first position after arrival
                     */
                    if (mecanumCommand.isPositionReached()) {
                        hw.intake.setPower(1.0);
                        mecanumCommand.stop();
                        mecanumCommand.moveToPos(112, -39, -Math.PI/2);
                        pusherTimer.reset();
                        if (mecanumCommand.isPositionReached() && pusherTimer.milliseconds() >= 500) {

                            hw.sorter.setPosition(SORTER_FIRST_POS);
                            pusherTimer.reset();

                        }
                        autoState = AUTO_STATE.INTAKE_ONE;
                    }
                    break;
                case INTAKE_ONE:
                    /**
                     * PHASE 6: Intake first game element
                     * Moves slightly forward to (112, -44) for element pickup
                     * Advances sorter to second position after intake
                     */
                    hw.intake.setPower(1.0);
                    hw.shooter.setPower(0);
                    hw.sorter.setPosition(SORTER_FIRST_POS);
                    mecanumCommand.moveToPos(112, -44, -Math.PI/2);

                    if (mecanumCommand.isPositionReached() && pusherTimer.milliseconds() >= 500) {
                        mecanumCommand.stop();
                        hw.sorter.setPosition(SORTER_SECOND_POS);
                        telemetry.addLine("the big sw");
                        autoState = AUTO_STATE.INTAKE_TWO;
                        pusherTimer.reset();


                    }

                    break;
                case INTAKE_TWO:
                    /**
                     * PHASE 7: Intake second game element
                     * Continues forward to (112, -49) for second element
                     * Longer delay (700ms) compared to first intake
                     */
                    hw.intake.setPower(1.0);
                    hw.sorter.setPosition(SORTER_SECOND_POS);
                    mecanumCommand.moveToPos(112, -49, -Math.PI/2);
                    if (mecanumCommand.isPositionReached() && pusherTimer.milliseconds() >= 700) {
                        mecanumCommand.stop();
                        hw.sorter.setPosition(SORTER_THIRD_POS);
                        autoState = AUTO_STATE.INTAKE_THREE;
                        pusherTimer.reset();


                    }

                    break;
                case INTAKE_THREE:
                    /**
                     * PHASE 8: Intake third element and final positioning
                     * Navigates to far position (80, 75) at 1.64 radians
                     * This is the final state - autonomous ends here
                     */
                    hw.intake.setPower(1.0);
                    hw.sorter.setPosition(SORTER_THIRD_POS);
                    mecanumCommand.moveToPos(80, 75, 1.64);

                    if (mecanumCommand.isPositionReached() && pusherTimer.milliseconds() >= 600) {
                        mecanumCommand.stop();
                        hw.sorter.setPosition(SORTER_THIRD_POS);
                        telemetry.addLine("the big sw");
                        pusherTimer.reset();


                    }
                    break;
                default:
                    mecanumCommand.stop();
                    break;


            }
            // Update telemetry every loop
            updateTelemetry();
        }
    }

    /**
     * Update driver station telemetry with current robot state.
     * Simplified compared to standard autonomous - only shows position data.
     */
    public void updateTelemetry() {
        telemetry.addData("x: ", mecanumCommand.getOdoX());
        telemetry.addData("y: ", mecanumCommand.getOdoY());
        telemetry.addData("Theta: ", mecanumCommand.getOdoHeading());


        telemetry.update();
    }


}



