/**
 * ===============================================================================
 * FIRST TECH CHALLENGE -  BLUE ALLIANCE AUTONOMOUS PROGRAM
 * ===============================================================================
 *
 * FILE: Blue.java
 * TEAM: Pr0Teens (FTC Team)
 * SEASON: 2025-2026
 *
 * DESCRIPTION:
 * This autonomous program is designed for the Blue Alliance starting position.
 * It executes a complex multi-phase autonomous routine that includes:
 *   - Preload specimen scoring (3 game elements)
 *   - Strategic repositioning and intake operations
 *   - Six-ball scoring sequence with precise positioning
 *   - Optional nine-ball extended autonomous (currently disabled)
 *
 * SYSTEM ARCHITECTURE:
 * - Uses state machine pattern with AUTO_STATE enum for phase management
 * - Implements stage-based control within each state for precise sequencing
 * - Integrates multiple subsystems: Mecanum drivetrain, intake, outtake, turret
 * - Employs PID control for accurate positioning using Pinpoint odometry
 *
 * KEY FEATURES:
 * - Autonomous navigation using field-relative coordinates
 * - Timed servo operations for game element manipulation
 * - Multi-position sorter mechanism for sequential shooting
 * - Velocity-controlled shooter with feedforward compensation
 *
 * ACKNOWLEDGMENTS:
 * Special thanks to Kirby for contributions to this codebase.
 * ===============================================================================
 *///

package org.firstinspires.ftc.teamcode.opmodes.tests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.cameras.IntakeSubsystem;


import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.odometry.PinPointOdometrySubsystem;


@Autonomous(name = "Blue Auto")
public class Blue extends LinearOpMode {
    // Hardware instance - singleton pattern for robot hardware management
    private Hardware hw;
    // Core timing system for autonomous sequencing
    ElapsedTime timer;
    // Subsystem command objects for modular control architecture
    private MecanumCommand mecanumCommand;
    private TurretSubsystem turretSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private OuttakeCommand outtakeCommand;
    private PinPointOdometrySubsystem odo;
    // State management flag for timing operations
    boolean firstInstance = true;
    /**
     * State machine enumeration defining all autonomous phases.
     * Each state represents a distinct phase of the autonomous routine.
     */
    enum AUTO_STATE {
        MOVEPRELOAD,
        TURNPRELOAD,
        PRELOAD_ONE,
        PRELOAD_TWO,
        PRELOAD_THREE,
        PRELOAD_EMPTY,
        INTAKE_ONE,
        INTAKE_MOVE,
        INTAKE_TWO,
        INTAKE_THREE,
        INTAKE_SHOOT,
        INTAKE_SHOOT1,
        INTAKE_SHOOT2,
        SIX_BALL,
        INTAKE_MOVE2,
        INTAKE_ONE2,
        INTAKE_TWO2,
        INTAKE_THREE2,
        NINE_BALL,
        LEAVE,


    }

    // Initialize autonomous to first state
    AUTO_STATE autoState = AUTO_STATE.MOVEPRELOAD;

    // Servo position constants for pusher mechanism (dual pusher design)
    private static final double PUSHER_UP = 0.18;
    private static final double PUSHER_DOWN = 0.0;
    private static final double PUSHER_UP1 = 0.18;
    private static final double PUSHER_DOWN1 = 0;
    private static final long PUSHER_TIME = 300;

    // Timing system for pusher operations
    private final ElapsedTime pusherTimer = new ElapsedTime();
    private boolean delayTimerStarted = false;

    // Pusher state tracking
    private boolean isPusherUp = false;

    // PID tuning constants for positional control
    // X-axis PID gains
    public static double kpx = 0.055;
    public static double kpy = 0.055;
    public static double kdx = 0.0029;
    public static double kdy = 0.0029;
    // Rotational PID gains
    public static double kpTheta = 1.45;
    public static double kdTheta = 0.0095;
    // Integral gains (currently disabled with kix=kiy=0)
    public static double kix = 0;
    public static double kiy = 0;
    public static double kitheta = 40000;
    // Sorter mechanism control
    int sorterpos = 0;
    private static final double SORTER_FIRST_POS = 0.0;
    private static final double SORTER_SECOND_POS = 0.45;
    private static final double SORTER_THIRD_POS = 0.90;
    // Direct hardware references for critical components
    Servo pusher;
    Servo pusher1;
    Servo sorter;
    Servo stopper;

    // State machine stage tracking (sub-states within each AUTO_STATE)
    private int stage1 = 0;
    // Legacy stage variable (appears unused)
    double stage = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware singleton and subsystems
        Hardware hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);

        intakeSubsystem = new IntakeSubsystem(hw);

        turretSubsystem = new TurretSubsystem(hw);
        outtakeCommand = new OuttakeCommand(hw);
        odo = new PinPointOdometrySubsystem(hw);

        // Initialize timing system
        timer = new ElapsedTime();
        // Configure shooter motor to use encoder for velocity control
        hw.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Set motor directions for proper operation
        hw.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // Apply PID constants to mecanum drive system
        mecanumCommand.setConstants(kpx, kdx, kix,
                kpy, kdy, kiy,
                kpTheta, kdTheta, kitheta);

        // Initialize shooter velocity control parameters
        ElapsedTime timer = new ElapsedTime();
        double kp = 0.8;
        double kf = 0.002;
        boolean paused = false;
        boolean submersibleTargetSet = false;
        boolean motorflag = true;
        double target = 2500;
        double current = hw.shooter.getVelocity();
        double error = target - current;

        double power = kp * error + kf * target;

        // Optional: LED indicator control (currently disabled)
//        hw.light.setPosition(0);

        // Get servo hardware references directly from hardware map
        pusher = hardwareMap.get(Servo.class, "pusher");
        pusher1 = hardwareMap.get(Servo.class, "pusher1");
        sorter = hardwareMap.get(Servo.class, "sorter");
        stopper = hardwareMap.get(Servo.class, "stopper");
        // Initialize servos to starting positions
        pusher.setPosition(0.0);
        pusher1.setPosition(0.0);
        sorter.setPosition(0);
        // Wait for driver station start signal
        waitForStart();
        // Reset timer and configure servo direction
        pusherTimer.reset();
        pusher1.setDirection(Servo.Direction.REVERSE);
        sorter.setPosition(SORTER_FIRST_POS);
        // Optional: Reset odometry (currently disabled)
//        odo.reset();

        // Main autonomous loop - runs until stop is requested
        while (opModeIsActive()) {
            // Update driver station telemetry
            updateTelemetry();

            // Core subsystem processing (must run every loop)
            mecanumCommand.motorProcess();
            mecanumCommand.processPIDUsingPinpoint();
            mecanumCommand.deadReckoning();
            mecanumCommand.processOdometry();

            // State machine - execute current autonomous phase
            switch (autoState) {
                case MOVEPRELOAD:
                    processMovePreload();
                    break;
                case PRELOAD_ONE:
                    processPreloadOne();
                    break;
                case PRELOAD_TWO:
                    processPreloadTwo();
                    break;
                case PRELOAD_THREE:
                    processPreloadThree();
                    break;
                case INTAKE_MOVE:
                    processIntakeMove();
                    break;
                case INTAKE_ONE:
                    processIntakeOne();
                    break;
                case INTAKE_TWO:
                    processIntakeTwo();
                    break;
                case INTAKE_THREE:
                    processIntakeThree();
                    break;
                case SIX_BALL:
                    processSixBall();
                    break;
                case INTAKE_MOVE2:
                    intakeMove2();
                    break;
                // Disabled states (commented out for this autonomous configuration)
//                case INTAKE_ONE2:
//                    intakeOne2();
//                    break;
//                case INTAKE_TWO2:
//                    intakeTwo2();
//                    break;
//                case INTAKE_THREE2:
//                    intakeThree2();
//                    break;
//                case NINE_BALL:
//                    nineBall();
//                    break;
//                case LEAVE:
//                    leave();
//                    break;


                default:
                    updateTelemetry();
                    break;
            }
        }
    }

    /**
     * PHASE 1: Move to preload scoring position
     * Navigates to the scoring position while activating intake and spinning up shooter.
     */
    private void processMovePreload() {
        intakeSubsystem.intake();
        outtakeCommand.spinup();
        switch (stage1) {
            case 0:
                stopper.setPosition(0.5);
                mecanumCommand.moveToPos(85, 58, 2.27);
                if (mecanumCommand.isPositionReached()) {
                    stage1++;
                }

                break;
            case 1:
                stopper.setPosition(0.5);
                pusher.setPosition(PUSHER_UP);
                pusher1.setPosition(PUSHER_UP1);
                waitTime(300);
                break;
            case 2:
                pusher.setPosition(PUSHER_DOWN);
                pusher1.setPosition(PUSHER_DOWN1);
                waitTime(300);
            case 3:
                stage1 = 0;
                autoState = AUTO_STATE.PRELOAD_ONE;
        }
    }
    /**
     * PHASE 2: Score first preloaded specimen
     * Performs timed pusher sequence and advances sorter to second position.
     */
    private void processPreloadOne() {
        switch (stage1) {
            case 0:
                waitTime(300);
                stopper.setPosition(0.5);
                break;
            case 1:
                pusher.setPosition(PUSHER_UP);
                pusher1.setPosition(PUSHER_UP1);
                waitTime(300);
                break;
            case 2:
                pusher.setPosition(PUSHER_DOWN);
                pusher1.setPosition(PUSHER_DOWN1);
                waitTime(300);
                break;
            case 3:
                sorter.setPosition(SORTER_SECOND_POS);
                waitTime(500);
                break;

            case 4:
                autoState = AUTO_STATE.PRELOAD_TWO;
                //wait time
                stage1 = 0;
                break;


        }


    }
    /**
     * PHASE 3: Score second preloaded specimen
     * Similar sequence to PRELOAD_ONE with sorter advancement.
     */
    private void processPreloadTwo() {

//        hw.shooter.setVelocityPIDFCoefficients(67, 0, 0, 0);
        outtakeCommand.spinup();
        outtakeCommand.spinup();
        stopper.setPosition(0.5);
        switch (stage1) {
//            case 0:
//                pusher.setPosition(PUSHER_UP);
//                pusher1.setPosition(PUSHER_UP1);
//                waitTime(200);
//                break;
//            case 1:
//
//                pusher.setPosition(PUSHER_DOWN);
//                pusher1.setPosition(PUSHER_DOWN1);
//                waitTime(500);
//                //wait time
//
//                break;
            case 0:
                pusher.setPosition(PUSHER_UP);
                pusher1.setPosition(PUSHER_UP1);
                waitTime(300);
                break;
            case 1:

                pusher.setPosition(PUSHER_DOWN);
                pusher1.setPosition(PUSHER_DOWN1);
                waitTime(300);
                //wait time

                break;
            case 2:
                sorter.setPosition(SORTER_THIRD_POS);
                waitTime(500);
                break;

            case 3:
                autoState = AUTO_STATE.PRELOAD_THREE;
                // wait time
                stage1 = 0;
                break;


        }
    }

    /**
     * PHASE 4: Score third preloaded specimen
     * Final preload scoring followed by transition to intake phase.
     */
    private void processPreloadThree() {

//        hw.shooter.setVelocityPIDFCoefficients(67, 0, 0, 0);
        outtakeCommand.spinup();
        stopper.setPosition(0.5);
        switch (stage1) {
            case 0:
                pusher.setPosition(PUSHER_UP);
                pusher1.setPosition(PUSHER_UP1);
                waitTime(500);

                //wait time
                break;
            case 1:
                pusher.setPosition(PUSHER_DOWN);
                pusher1.setPosition(PUSHER_DOWN1);
                waitTime(500);
                //wait time

                break;
            case 2:
                sorter.setPosition(SORTER_FIRST_POS);
                waitTime(500);
                break;
            case 3:
                autoState = AUTO_STATE.INTAKE_MOVE;
                // wait time
                stage1 = 0;
                break;

        }

    }
    /**
     * PHASE 5: Navigate to intake position
     * Moves robot to field position for collecting game elements.
     */
    private void processIntakeMove() {
        intakeSubsystem.intake();
        outtakeCommand.stopShooter();
        stopper.setPosition(0);
        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(118, 40, Math.PI / 2);
                if (mecanumCommand.isPositionReached()) {
                    stage1++;
                }
                break;
            case 1:
                autoState = AUTO_STATE.INTAKE_ONE;
                stage1 = 0;
                break;


        }

    }
    /**
     * PHASE 6: Intake first game element
     * Precise positioning and timing for element collection.
     */
    private void processIntakeOne() {
        intakeSubsystem.intake();
        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(112, 15, Math.PI / 2);
                if (mecanumCommand.isPositionReached()) {
                    mecanumCommand.moveToPos(112, 15, Math.PI / 2);
                    stage1++;

                }
                break;
            case 1:
                waitTime(500);
                break;
            case 2:
                sorter.setPosition(SORTER_SECOND_POS);
                waitTime(800);
                break;
            case 3:
                autoState = AUTO_STATE.INTAKE_TWO;
                stage1 = 0;
                break;

        }


    }
    /**
     * PHASE 7: Intake second game element
     * Similar approach pattern with different Y-coordinate.
     */
    private void processIntakeTwo() {
        intakeSubsystem.intake();

        switch (stage1) {
            case 0:
                waitTime(400);
                mecanumCommand.moveToPos(112, 4, Math.PI / 2);
                if (mecanumCommand.isPositionReached()) {
                    mecanumCommand.moveToPos(112, 5.5, Math.PI / 2);
                    stage1++;
                }

                break;
            case 1:
                waitTime(500);
                break;
            case 2:
                sorter.setPosition(SORTER_THIRD_POS);
                waitTime(800);
                break;
            case 3:
                autoState = AUTO_STATE.INTAKE_THREE;
                stage1 = 0;
                break;

        }


    }
    /**
     * PHASE 8: Intake third game element
     * Final intake collection before returning to scoring position.
     */
    private void processIntakeThree() {
        intakeSubsystem.intake();
        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(112, -8, Math.PI / 2);
                if (mecanumCommand.isPositionReached()) {
                    mecanumCommand.moveToPos(112, -20, Math.PI / 2);
                    stage1++;
                }
                break;
            case 1:
                waitTime(500);
                break;
            case 2:
                stage1 = 0;
                autoState = AUTO_STATE.SIX_BALL;
                break;
        }


    }
    /**
     * PHASE 9: Six-ball scoring sequence
     * Returns to scoring position and executes rapid-fire scoring of all collected elements.
     * This is the climax of the autonomous routine.
     */
    private void processSixBall() {
        outtakeCommand.spinup();
        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(87, 58, 2.27);
                if (mecanumCommand.isPositionReached()) {
                    mecanumCommand.stop();
                    sorter.setPosition(SORTER_FIRST_POS);
                    stage1++;
                }

                break;
            case 1:
                waitTime(300);
                stopper.setPosition(0.5);
                break;
            case 2:
                pusher.setPosition(PUSHER_UP);
                pusher1.setPosition(PUSHER_UP1);
                waitTime(500);
                break;
            case 3:
                pusher.setPosition(PUSHER_DOWN);
                pusher1.setPosition(PUSHER_DOWN1);
                waitTime(300);
                break;
            case 4:
                sorter.setPosition(SORTER_SECOND_POS);
                waitTime(500);
                break;
            case 5:
                pusher.setPosition(PUSHER_UP);
                pusher1.setPosition(PUSHER_UP1);
                waitTime(500);
                break;
            case 6:
                pusher.setPosition(PUSHER_DOWN);
                pusher1.setPosition(PUSHER_DOWN1);
                waitTime(300);
                break;
            case 7:
                sorter.setPosition(SORTER_THIRD_POS);
                waitTime(500);
                break;
            case 8:
                pusher.setPosition(PUSHER_UP);
                pusher1.setPosition(PUSHER_UP1);
                waitTime(500);
                break;
            case 9:
                pusher.setPosition(PUSHER_DOWN);
                pusher1.setPosition(PUSHER_DOWN1);
                waitTime(300);
                break;
            case 10:
                pusher.setPosition(PUSHER_UP);
                pusher1.setPosition(PUSHER_UP1);
                waitTime(500);
                break;
            case 11:
                pusher.setPosition(PUSHER_DOWN);
                pusher1.setPosition(PUSHER_DOWN1);
                waitTime(300);
                break;
            case 12:
                autoState = AUTO_STATE.INTAKE_MOVE2;
                outtakeCommand.stopShooter();
                stage1 = 0;
                break;

        }
    }
    /**
     * PHASE 10: Second intake movement (optional extension)
     * Navigate to alternate intake position for extended autonomous.
     * Currently ends autonomous - further intake states are disabled.
     */
    private void intakeMove2() {
        intakeSubsystem.intake();
        stopper.setPosition(0);
        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(112, -8, Math.PI / 2);
                if (mecanumCommand.isPositionReached()) {
                    mecanumCommand.stop();
                    stage1++;
                }
                break;
            case 1:
//                autoState = AUTO_STATE.INTAKE_ONE2;
                stage1 = 0;
                break;


        }

    }
    /**
     * DISABLED PHASE: Second intake cycle - first element
     * This method is part of an optional nine-ball autonomous extension.
     * Currently disabled to keep autonomous runtime within time limits.
     */
    private void intakeOne2() {
        intakeSubsystem.intake();
        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(180, 16, Math.PI / 2);
                if (mecanumCommand.isPositionReached()) {
                    mecanumCommand.moveToPos(180, 18, Math.PI / 2);

                    stage1++;

                }
                break;
            case 1:
                waitTime(300);
                break;
            case 2:
                sorter.setPosition(SORTER_SECOND_POS);
                waitTime(500);
                break;
            case 3:
                autoState = AUTO_STATE.INTAKE_TWO2;
                stage1 = 0;
                break;

        }


    }
    /**
     * DISABLED PHASE: Second intake cycle - second element
     */
    private void intakeTwo2() {
        intakeSubsystem.intake();

        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(180, 10, Math.PI / 2);
                if (mecanumCommand.isPositionReached()) {
                    mecanumCommand.moveToPos(180, 12, Math.PI / 2);
                    stage1++;
                }

                break;
            case 1:
                waitTime(300);//changed delay
                break;
            case 2:
                sorter.setPosition(SORTER_THIRD_POS);
                waitTime(600);
                break;
            case 3:
                autoState = AUTO_STATE.INTAKE_THREE2;
                stage1 = 0;
                break;

        }


    }
    /**
     * DISABLED PHASE: Second intake cycle - third element
     */
    private void intakeThree2() {
        intakeSubsystem.intake();
        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(180, -9, Math.PI / 2);
                if (mecanumCommand.isPositionReached()) {
                    mecanumCommand.moveToPos(180, -7, Math.PI / 2);
                    stage1++;
                }
                break;
            case 1:
                waitTime(400);
                break;
            case 2:
                stage1 = 0;
                autoState = AUTO_STATE.NINE_BALL;
                break;


        }


    }
    /**
     * DISABLED PHASE: Nine-ball scoring sequence
     * Extended autonomous scoring all nine collected elements.
     */
    private void nineBall() {
        outtakeCommand.spinup();
        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(80, 48, 3 * (Math.PI / 4));
                if (mecanumCommand.isPositionReached()) {
                    sorter.setPosition(SORTER_FIRST_POS);
                    stage1++;
                }

                break;
            case 1:
                waitTime(300);
                stopper.setPosition(0.5);
                break;
            case 2:
                pusher.setPosition(PUSHER_UP);
                pusher1.setPosition(PUSHER_UP1);
                waitTime(200);
                break;
            case 3:
                pusher.setPosition(PUSHER_DOWN);
                pusher1.setPosition(PUSHER_DOWN1);
                waitTime(200);
                break;
            case 4:
                sorter.setPosition(SORTER_SECOND_POS);
                waitTime(500);
                break;

            case 5:
                pusher.setPosition(PUSHER_UP);
                pusher1.setPosition(PUSHER_UP1);
                waitTime(200);
                break;
            case 6:
                pusher.setPosition(PUSHER_DOWN);
                pusher1.setPosition(PUSHER_DOWN1);
                waitTime(200);
                break;
            case 7:
                sorter.setPosition(SORTER_THIRD_POS);
                waitTime(500);
                break;
            case 8:
                pusher.setPosition(PUSHER_UP);
                pusher1.setPosition(PUSHER_UP1);
                waitTime(200);
                break;
            case 9:
                pusher.setPosition(PUSHER_DOWN);
                pusher1.setPosition(PUSHER_DOWN1);
                waitTime(200);
                break;
            case 10:
                autoState = AUTO_STATE.LEAVE;
                stage1 = 0;
                break;
        }


    }
    /**
     * DISABLED PHASE: Final repositioning
     * Moves robot to neutral position after scoring.
     */

    private void leave() {
        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(180, 22, Math.PI / 2);
                if (mecanumCommand.isPositionReached()) {
                    mecanumCommand.stop();

                }

        }
    }

    /**
     * Update driver station telemetry with current robot state.
     * Provides real-time feedback for debugging and monitoring.
     */
    public void updateTelemetry() {
        telemetry.addData("x: ", mecanumCommand.getOdoX());
        telemetry.addData("y: ", mecanumCommand.getOdoY());
        telemetry.addData("Thetareading: ", mecanumCommand.getOdoHeading());
        telemetry.addData("state: ", autoState);
        telemetry.addData("stage: ", stage1);
        telemetry.addData("xFinal: ", mecanumCommand.xFinal);
        telemetry.addData("yFinal: ", mecanumCommand.yFinal);
        telemetry.addData("thetaFinal: ", mecanumCommand.thetaFinal);

        telemetry.update();
    }


    private void waitTime(double milliseconds) {
        if (firstInstance) {
            timer.reset();
            firstInstance = false;
        }
        if (timer.milliseconds() > milliseconds && !isStopRequested()) {
            firstInstance = true;
            stage1++;
        }
    }

}






