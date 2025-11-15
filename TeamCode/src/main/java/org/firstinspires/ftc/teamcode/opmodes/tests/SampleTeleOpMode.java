package org.firstinspires.ftc.teamcode.opmodes.tests;

import static org.firstinspires.ftc.teamcode.subsystems.cameras.LogitechSubsystem.obelisk;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.odometry.PinPointOdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.cameras.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.cameras.LogitechSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.sorting.SortingSubsystem;


@TeleOp(name = "TeleopSample", group = "TeleOp")
public class SampleTeleOpMode extends LinearOpMode {

    // opmodes should only own commands
    private MecanumCommand mecanumCommand;
    private OuttakeCommand outtakeCommand;
    private LimelightSubsystem limelightsub;
    private LogitechSubsystem logitechsub;
    private SortingSubsystem sortingSubsystem;
    private ElapsedTime timer;
    private Hardware hw;
    private ElapsedTime resetTimer;

    // --- Button Variables ---
    private boolean previousAState = false;
    private boolean previousBState = false;
    private boolean previousXState = false;
    private boolean previousYState = false;

    private boolean previousLBumpState = false;

    // --- Intake/Outake Variables---
    private boolean isIntakeMotorOn = false;
    private boolean isOuttakeMotorOn = false;

    // --- Pusher Variables ---
    private static final double PUSHER_UP = 0.18;
    private static final double PUSHER_DOWN = 0;
    private static final long PUSHER_TIME = 750;
    private final ElapsedTime pusherTimer = new ElapsedTime();

    private boolean isPusherUp = false;

    // --- Sorter Variables ---
    private final ElapsedTime sorterTimer = new ElapsedTime();
    int sorterpos = 0;
    private static final double SORTER_FIRST_POS = 0.0;
    private static final double SORTER_SECOND_POS = 0.38;
    private static final double SORTER_THIRD_POS = 0.78;

    boolean spunUp;
    int counter = 1;
    private String ALLIANCE = "blue";
    private String motif;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        outtakeCommand = new OuttakeCommand(hw);
        limelightsub = new LimelightSubsystem(hw, telemetry);
        sortingSubsystem = new SortingSubsystem(hw, telemetry, motif);

        hw.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        resetTimer = new ElapsedTime();
        hw.pusher.setPosition(PUSHER_DOWN);
        hw.sorter.setPosition(0);
        hw.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeCommand.setMaxRPM(5000);
        hw.light.setPosition(0);

        while (opModeInInit()) {
            if (gamepad1.b) {
                ALLIANCE = "red";
            }
            if (gamepad1.x) {
                ALLIANCE = "blue";
            }
            telemetry.addData("Alliance: ", ALLIANCE);
            telemetry.update();
        }
        // Wait for start button to be pressed
        waitForStart();


        logitechsub = new LogitechSubsystem(hw, ALLIANCE);

        // Loop while OpMode is running
        while (opModeIsActive()) {
            logitechsub.pattern();
            logitechsub.telemetryAprilTag(telemetry);

//            if (obelisk == "PPG"){
//
//            } else if (obelisk == "PGP"){
//
//            } else if (obelisk == "GPP"){
//
//            }

            mecanumCommand.processOdometry();
            mecanumCommand.normalMove(
                    gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );

            processTelemetry();

            if (gamepad1.start) {
                mecanumCommand.resetPinPointOdometry();
            }

            boolean currentLBumpState = gamepad1.left_bumper;
            if (currentLBumpState && !previousLBumpState) {
                hw.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                isIntakeMotorOn = !isIntakeMotorOn;
                hw.intake.setPower(isIntakeMotorOn ? 1.0 : 0.0);
            }
            previousLBumpState = currentLBumpState;

            // --- Intake toggle on A (edge) ---
            boolean currentAState = gamepad1.a;
            if (currentAState && !previousAState) {
                hw.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                isIntakeMotorOn = !isIntakeMotorOn;
                hw.intake.setPower(isIntakeMotorOn ? 1.0 : 0.0);
            }
            previousAState = currentAState;

            // --- Pusher pulse on Y (edge) ---
            boolean currentYState = gamepad1.y;
            if (currentYState && !previousYState) {
                // Start pulse only if not already pulsing
                if (!isPusherUp) {
                    hw.pusher.setPosition(PUSHER_UP);
                    pusherTimer.reset();
                    isPusherUp = true;
                }
            }
            previousYState = currentYState;

            // Pusher
            if (isPusherUp && pusherTimer.milliseconds() >= PUSHER_TIME) {
                hw.pusher.setPosition(PUSHER_DOWN);
                isPusherUp = false;
            }

            // Outtake
            boolean currentXState = gamepad1.x;


            if (currentXState && !previousXState) {
                isOuttakeMotorOn = !isOuttakeMotorOn;
            }
            previousXState = currentXState;

            if (isOuttakeMotorOn) {
                spunUp = outtakeCommand.spinup();
                lightOn(spunUp);
            } else if (!isOuttakeMotorOn) {
                outtakeCommand.stopShooter();
                hw.light.setPosition(0);
            }

            boolean currentBState = gamepad1.b;
            if (currentBState && !previousBState) {
                limelightsub.ballPosition(telemetry, mecanumCommand);
            }
            previousBState = currentBState;

            if (gamepad1.b && sorterTimer.milliseconds() > 800 && !isPusherUp) {
                sorterTimer.reset();
                if (sorterpos == 0) {
                    hw.sorter.setPosition(SORTER_FIRST_POS);//60 degrees
                } else if (sorterpos == 1) {
                    hw.sorter.setPosition(SORTER_SECOND_POS);//60 degrees
                } else if (sorterpos == 2) {
                    hw.sorter.setPosition(SORTER_THIRD_POS);//60 degrees
                }
                sorterpos = (sorterpos + 1) % 3;
            }

            if (gamepad1.right_bumper && sorterTimer.milliseconds() >= 500) {
                telemetry.addLine("running Sort code");
                sorterTimer.reset();
                sortingSubsystem.temporarySort();
            }
        }

    }

    public void processTelemetry() {
        //add telemetry messages here
        //telemetry.addData("resetTimer: ",  resetTimer.milliseconds());
        telemetry.addLine("---------------------------------");
        telemetry.addData("X", mecanumCommand.getX());
        telemetry.addData("Y", mecanumCommand.getY());
        telemetry.addData("Pusher ON", isPusherUp);
        telemetry.addData("Pattern ", logitechsub.pattern());
        telemetry.addData("TPS: ", hw.shooter.getVelocity());
        telemetry.addData("Pattern ", obelisk);
        telemetry.addData("light:  ", spunUp);
        telemetry.addData("x ", logitechsub.targetApril());
        telemetry.update();
    }

    public void lightOn(boolean rpmReached) {
        if (rpmReached && isOuttakeMotorOn) {
            hw.light.setPosition(0.5);
        } else {
            hw.light.setPosition(0);
        }


    }
}