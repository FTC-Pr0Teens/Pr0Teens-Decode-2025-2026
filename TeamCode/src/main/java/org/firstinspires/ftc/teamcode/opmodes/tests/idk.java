package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.cameras.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.cameras.LogitechSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.sorting.SortingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.cameras.LogitechSubsystem;

@TeleOp(name = "idk", group = "TeleOp")
public class idk extends LinearOpMode {

    private MecanumCommand mecanumCommand;
    private OuttakeCommand outtakeCommand;
    private LogitechSubsystem logitechsub;

    private Hardware hw;

    // --- Button Variables ---
    private boolean previousAState = false;
    private boolean previousBState = false;
    private boolean previousXState = false;
    private boolean previousYState = false;
    private boolean previousLBumpState = false;
    private boolean isIntakeMotorOn = false;
    private boolean isOuttakeMotorOn = false;

    private static final double PUSHER_UP = 0.39;
    private static final double PUSHER_DOWN = 0.0;
    private static final double PUSHER_UP1 = 0.19;
    private static final double PUSHER_DOWN1 = 0;
    private static final long PUSHER_TIME = 750;

    private final ElapsedTime pusherTimer = new ElapsedTime();
    private final ElapsedTime sorterTimer = new ElapsedTime();

    private boolean isPusherUp = false;

    int sorterpos = 0;
    private static final double SORTER_FIRST_POS = 0.0;
    private static final double SORTER_SECOND_POS = 0.45;
    private static final double SORTER_THIRD_POS = 0.90;
    private String ALLIANCE = "blue";
    private boolean autoAimState = false;
    private boolean previousAimButton = false;
    boolean runPusher = false;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        outtakeCommand = new OuttakeCommand(hw);
        logitechsub = new LogitechSubsystem(hw, ALLIANCE);
        outtakeCommand = new OuttakeCommand(hw);


        hw.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        hw.pusher1.setDirection(Servo.Direction.REVERSE);
        hw.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtakeCommand.setMaxRPM(3000);

        while (opModeInInit()) {
            if (gamepad1.b) {
                ALLIANCE = "red";
            }
            if (gamepad1.x) {
                ALLIANCE = "blue";
            }
            hw.pusher.setPosition(0);
            hw.pusher1.setPosition(0);
            hw.sorter.setPosition(0);
//            hw.light.setPosition(0);
        }

        waitForStart();

        // Loop while OpMode is running
        while (opModeIsActive()) {
            processTelemetry();

            mecanumCommand.normalMove(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            //Intake backwards
            boolean currentLBumpState = gamepad1.left_bumper;
            if (currentLBumpState && !previousLBumpState) {
                hw.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                isIntakeMotorOn = !isIntakeMotorOn;
                hw.intake.setPower(isIntakeMotorOn ? 1.0 : 0.0);
            }
            previousLBumpState = currentLBumpState;

            // --- Intake toggle on A ---
            boolean currentAState = gamepad1.a;
            if (currentAState && !previousAState) {
                hw.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                isIntakeMotorOn = !isIntakeMotorOn;
                hw.intake.setPower(isIntakeMotorOn ? 1.0 : 0.0);
            }
            previousAState = currentAState;

            boolean currentXState = gamepad1.x;
            if (currentXState && !previousXState) {
                isOuttakeMotorOn = !isOuttakeMotorOn;
            }
            previousXState = currentXState;

            if (isOuttakeMotorOn){
                outtakeCommand.setMaxRPM(3000);
                outtakeCommand.spinup();
            } else {
                outtakeCommand.stopShooter();
            }

            // --- Pusher up on Y  ---
            boolean currentYState = gamepad1.y;

            if (gamepad1.y && !runPusher){
                runPusher = true;
            }

            if (runPusher){
                runPusher = outtakeCommand.transfer();
            }



            if (gamepad1.right_bumper && !previousAimButton) {
                autoAimState = !autoAimState;   // toggle on *edge* of button press
            }
            previousAimButton = gamepad2.right_bumper;

            if (autoAimState) {
                if (logitechsub.targetApril() > 5) {
                    mecanumCommand.pivot(0.2);
                } else if (logitechsub.targetApril() < -5) {
                    mecanumCommand.pivot(-0.2);
                } else {
                    mecanumCommand.pivot(0);
                }
            }

            if (isOuttakeMotorOn) {
                if (logitechsub.distance() >= 100) {
                    outtakeCommand.setMaxRPM(3500);
                } else if (logitechsub.distance() <= 90 && logitechsub.distance() >= 35) {
                    outtakeCommand.setMaxRPM(2800);
                } else if (logitechsub.distance() <= 35) {
                    outtakeCommand.setMaxRPM(2300);
                }
            }



        }
    }
    public void processTelemetry() {
        telemetry.addData("RPM", hw.shooter.getVelocity() * 60.0 / 28.0);
        telemetry.addData("x", logitechsub.targetApril());
        telemetry.addData("y", logitechsub.distance());
        telemetry.update();
    }
}
