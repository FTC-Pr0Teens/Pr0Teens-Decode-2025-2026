/**
 * ===============================================================================
 * FIRST TECH CHALLENGE - MAIN TELEOP CONTROL PROGRAM
 * ===============================================================================
 *
 * FILE: Pr0teensTeleop.java
 * TEAM: Pr0Teens (FTC Team)
 * SEASON: 2025-2026
 *
 * DESCRIPTION:
 * This is the primary driver-controlled (TeleOp) program for the robot. It provides
 * comprehensive control of all robot subsystems during the driver-controlled period
 * of FTC matches. The program features:
 *   - Field-oriented mecanum drive control
 *   - Toggle-based intake/outtake operations
 *   - Vision-assisted shooting with Limelight tracking
 *   - Automated turret alignment to AprilTag targets
 *   - Dynamic shooter velocity adjustment based on target distance
 *   - LED feedback for shooter readiness status
 *
 * CONTROL SCHEME (Gamepad 1):
 * - Left Stick Y:  Forward/backward translation
 * - Left Stick X:  Strafe left/right
 * - Right Stick X: Rotation
 * - A Button:      Toggle intake (forward)
 * - Left Bumper:   Toggle intake (reverse)
 * - X Button:      Toggle shooter system on/off
 * - Y Button:      Fire pusher mechanism (single press)
 * - B Button:      Manual sorter advancement
 * - Right Bumper:  Manual turret rotation (override auto-aim)
 *
 * INITIALIZATION CONTROLS:
 * - B Button (during init): Set alliance to RED
 * - X Button (during init): Set alliance to BLUE
 *
 * VISION SYSTEM:
 * - Uses Limelight 3A for AprilTag detection and ranging
 * - Automatic distance-based shooter velocity calculation
 * - Proportional turret control for target tracking (kp = 0.03)
 * - LED indicator shows shooter ready state (green when at target RPM)
 *
 * KEY FEATURES:
 * - Alliance-aware targeting (configured during initialization)
 * - Automatic shooter spin-up with RPM feedback
 * - Dead-zone for turret auto-aim (±2 degrees prevents oscillation)
 * - Toggle-based controls for clean state management
 * ===============================================================================
 */
package org.firstinspires.ftc.teamcode.opmodes.tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.cameras.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeCommand;

@TeleOp(name = "Pr0teens Teleop", group = "TeleOp")
public class Pr0teensTeleop extends LinearOpMode {
    // Subsystem command objects
    private MecanumCommand mecanumCommand;
    private OuttakeCommand outtakeCommand;
    // Hardware instance
    private Hardware hw;

    // Button state tracking for edge detection (prevents button holding issues)
    private boolean previousAState = false;
    private boolean previousBState = false;
    private boolean previousXState = false;
    private boolean lastYState = false;
    private boolean previousLBumpState = false;
    // Motor state flags
    private boolean isIntakeMotorOn = false;
    private boolean isOuttakeMotorOn = false;
    // Pusher servo position constants
    private static final double PUSHER_UP = 1;
    private static final double PUSHER_DOWN = 0.0;
    private static final double PUSHER_UP1 = -0.5;
    private static final double PUSHER_DOWN1 = 0;
    private static final long PUSHER_TIME = 750;

    // Sorter mechanism positions
    int sorterpos = 0;
    private static final double SORTER_FIRST_POS = 0.0;
    private static final double SORTER_SECOND_POS = 0.45;
    private static final double SORTER_THIRD_POS = 0.90;
    // Alliance selection and vision control
    private String ALLIANCE = "blue";
    private boolean augittoAimState = false;
    private boolean previousAimButton = false;
    private boolean runPusher = false;
    // Vision and control systems
    private LimelightSubsystem limelightsub;
    private double power;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware and subsystems
        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        outtakeCommand = new OuttakeCommand(hw);
        // Configure motor directions for correct operation
        hw.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        hw.turret.setDirection(DcMotorSimple.Direction.FORWARD);
        // Set shooter motors to velocity control mode
        hw.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /**
         * INITIALIZATION LOOP
         * Runs while waiting for driver station start command.
         * Allows drivers to select alliance color and initializes servos.
         */
        while (opModeInInit()) {
            // Alliance selection controls
            if (gamepad1.b) {
                ALLIANCE = "red";    // B button selects RED alliance
            }
            if (gamepad1.x) {
                ALLIANCE = "blue";   // X button selects BLUE alliance
            }
            // Initialize servos to safe starting positions
//            hw.sorter.setPosition(0);
//            hw.pusher.setPosition(PUSHER_DOWN);
//            hw.pusher1.setPosition(PUSHER_DOWN1);
//          // Alternative pusher starting position (commented out)
             hw.pusher.setPosition(PUSHER_UP);
//             hw.pusher1.setPosition(PUSHER_UP1);

            // Configure stopper servo
            hw.stopper.setDirection(Servo.Direction.FORWARD);
            hw.stopper.setPosition(0.2);
//            hw.light.setPosition(0);

            // Display alliance selection
//            telemetry.addData("alliance", ALLIANCE);
//            telemetry.update();
        }

        // Initialize Limelight subsystem with selected alliance
        limelightsub = new LimelightSubsystem(hw, telemetry, ALLIANCE);
        // Wait for match start
        waitForStart();
        // Initialize teleop control states
        runPusher = false;
        lastYState = gamepad1.y;

        /**
         * MAIN TELEOP LOOP
         * Runs continuously during driver-controlled period
         */
        while (opModeIsActive()) {
//            processTelemetry();
            /**
             * MECANUM DRIVE CONTROL
             * Field-oriented control using left stick for translation,
             * right stick for rotation. Y-axis inverted for intuitive control.
             */
            mecanumCommand.normalMove(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            /**
             * REVERSE INTAKE CONTROL (Left Bumper)
             * Toggles intake motor in reverse direction for clearing jams
             * or ejecting game elements.
             */
            boolean currentLBumpState = gamepad1.left_bumper;
            if (currentLBumpState && !previousLBumpState) {
                hw.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                isIntakeMotorOn = !isIntakeMotorOn;
                hw.intake.setPower(isIntakeMotorOn ? 1.0 : 0.0);
            }
            previousLBumpState = currentLBumpState;

            /**
             * FORWARD INTAKE CONTROL (A Button)
             * Primary intake control - toggles intake motor on/off
             * in normal (forward) direction for collecting game elements.
             */
            boolean currentAState = gamepad1.a;
            if (currentAState && !previousAState) {
                hw.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                isIntakeMotorOn = !isIntakeMotorOn;
                hw.intake.setPower(isIntakeMotorOn ? 1.0 : 0.0);
            }
            previousAState = currentAState;
            /**
             * SHOOTER SYSTEM TOGGLE (X Button)
             * Master on/off for entire shooter system including:
             * - Shooter wheel velocity control
             * - Vision-guided turret aiming
             * - LED status indicators
             */
            boolean currentXState = gamepad1.x;
            if (currentXState && !previousXState) {
                isOuttakeMotorOn = !isOuttakeMotorOn;
            }
            previousXState = currentXState;

            hw.light.setPosition(0.28);
            /**
             * SHOOTER SYSTEM OPERATION
             * When enabled, performs:
             * 1. Vision-based distance calculation
             * 2. Dynamic RPM adjustment based on distance
             * 3. Automatic turret alignment to AprilTag
             * 4. LED feedback for ready state
             */
            if (gamepad1.right_bumper) {
                hw.turret.setPower(-1.0);
            } else if (isOuttakeMotorOn) {
                // Automatic shooter operation with vision assistance
                outtakeCommand.setMaxRPM(outtakeCommand.getShooterRPM(limelightsub.distance(telemetry)));

//                outtakeCommand.setMaxRPM(3000);
                // LED feedback: Green when at target RPM, orange otherwise
                if (outtakeCommand.isRPMReached() == true) {
                    hw.light.setPosition(0.5);
                } else {
                    hw.light.setPosition(0.28);
                }
                // Spin up shooter wheels to target velocity
                outtakeCommand.spinup();
                // Open stopper to allow game elements to feed
                hw.stopper.setDirection(Servo.Direction.FORWARD);
                hw.stopper.setPosition(0.5);
                /**
                 * AUTO-AIM TURRET CONTROL
                 * Proportional control to center on AprilTag target
                 * Dead-zone: ±2 degrees to prevent oscillation
                 * Control gain: 0.03 (kp)
                 */
                if (Math.abs(limelightsub.apriltag(telemetry)) > 2) {
                    // Calculate proportional correction power
                    power = 0.03 * limelightsub.apriltag(telemetry);
                    // Clamp power to safe limits [-1.0, 1.0]
                    power = Math.max(-1.0, Math.min(1.0, power));
                    hw.turret.setPower(-power);
                } else {
                    // Within dead-zone - hold position
                    hw.turret.setPower(0);
                }

            } else {
                // Shooter system disabled - stop all components
                outtakeCommand.stopShooter();
                hw.stopper.setPosition(0);
                hw.turret.setPower(0);
            }


            /**
             * PUSHER TRIGGER (Y Button)
             * Single button press triggers automated pusher sequence.
             * The outtakeCommand.transfer() method handles the timing
             * and returns false when sequence is complete.
             */

            if (gamepad1.y && !lastYState) {
                runPusher = true;
            }

            lastYState = gamepad1.y;
            // Run pusher sequence if active
            if (runPusher) {
                runPusher = outtakeCommand.transfer();
            }
            /**
             * MANUAL SORTER CONTROL (B Button)
             * Allows manual advancement of sorter position
             * for aligning game elements with shooter.
             */
            if (gamepad1.b) {
                outtakeCommand.sorter(true);
            } else {
                outtakeCommand.sorter(false);
            }

            telemetry.addData("RPM", hw.shooter.getVelocity() * 60.0 / 28.0);
            telemetry.addData("x", limelightsub.apriltag(telemetry));
            telemetry.addData("y", limelightsub.distance(telemetry));
            telemetry.addData("power", power);
            telemetry.addData("alliance", ALLIANCE);
            telemetry.update();
        }
    }

    /**
     * Update driver station telemetry with critical system information.
     * Provides real-time feedback on:
     * - Shooter RPM (converted from encoder ticks to RPM)
     * - Vision targeting data (X-offset and distance)
     * - Turret power output
     * - Selected alliance color
     */
//    public void processTelemetry() {
//        telemetry.addData("RPM", hw.shooter.getVelocity() * 60.0 / 28.0);
//        telemetry.addData("x", limelightsub.apriltag(telemetry));
//        telemetry.addData("y", limelightsub.distance(telemetry));
//        telemetry.addData("power", power);
//        telemetry.addData("alliance", ALLIANCE);
//        telemetry.update();
//
//    }
}
