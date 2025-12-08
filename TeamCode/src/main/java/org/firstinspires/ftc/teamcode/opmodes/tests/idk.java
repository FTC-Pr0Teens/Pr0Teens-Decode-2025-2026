package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.cameras.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.cameras.LogitechSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeCommand;

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
    private boolean lastYState = false;
    private boolean previousLBumpState = false;
    private boolean isIntakeMotorOn = false;
    private boolean isOuttakeMotorOn = false;

    private static final double PUSHER_UP = 0.39;
    private static final double PUSHER_DOWN = 0.0;
    private static final double PUSHER_UP1 = 0.19;
    private static final double PUSHER_DOWN1 = 0;
    private static final long PUSHER_TIME = 750;

    int sorterpos = 0;
    private static final double SORTER_FIRST_POS = 0.0;
    private static final double SORTER_SECOND_POS = 0.45;
    private static final double SORTER_THIRD_POS = 0.90;
    private String ALLIANCE = "blue";
    private boolean autoAimState = false;
    private boolean previousAimButton = false;
    private boolean runPusher = false;
    private LimelightSubsystem limelightsub;
    private double power;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        outtakeCommand = new OuttakeCommand(hw);
        limelightsub = new LimelightSubsystem(hw, telemetry, ALLIANCE);

        hw.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        hw.turret.setDirection(DcMotorSimple.Direction.FORWARD);
        hw.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeInInit()) {
            if (gamepad1.b) {
                ALLIANCE = "red";
            }
            if (gamepad1.x) {
                ALLIANCE = "blue";
            }

            hw.sorter.setPosition(0);
            hw.pusher.setPosition(PUSHER_DOWN);
            hw.pusher1.setPosition(PUSHER_DOWN1);
            hw.stopper.setDirection(Servo.Direction.FORWARD);
            hw.stopper.setPosition(0.2);
//            hw.light.setPosition(0);

            telemetry.addData("alliance", ALLIANCE);
            telemetry.update();
        }

        waitForStart();

        runPusher = false;
        lastYState = gamepad1.y;

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
                outtakeCommand.setMaxRPM(outtakeCommand.getShooterRPM(limelightsub.distance(telemetry)));
//                outtakeCommand.setMaxRPM(2400);
                outtakeCommand.spinup();
                hw.stopper.setDirection(Servo.Direction.FORWARD);
                hw.stopper.setPosition(0.5);

//                if (limelightsub.apriltag(telemetry) >= 2) { hw.turret.setPower(-0.1); } else if (limelightsub.apriltag(telemetry) >= -2) { hw.turret.setPower(0.1); } else { hw.turret.setPower(0); }
                    if (Math.abs(limelightsub.apriltag(telemetry)) > 2) {
                        power = 0.03 * limelightsub.apriltag(telemetry);
                        power = Math.max(-1.0, Math.min(1.0, power));
                        hw.turret.setPower(-power);
                    } else {
                        hw.turret.setPower(0);
                    }

            } else {
                outtakeCommand.stopShooter();
                hw.stopper.setPosition(0);
                hw.turret.setPower(0);
            }


            // --- Pusher up on Y  ---
            if (gamepad1.y && !lastYState) {
                runPusher = true;
            }

            lastYState = gamepad1.y;

            if (runPusher) {
                runPusher = outtakeCommand.transfer();
            }

            if (gamepad1.b) {
                outtakeCommand.sorter(true);
            } else {
                outtakeCommand.sorter(false);
            }

            if (gamepad1.right_bumper && !previousAimButton) {
                autoAimState = !autoAimState;
            }
            previousAimButton = gamepad1.right_bumper;

//            if (autoAimState) {
//                if (logitechsub.targetApril() > 5) {
//                    mecanumCommand.pivot(0.2);
//                } else if (logitechsub.targetApril() < -5) {
//                    mecanumCommand.pivot(-0.2);
//                } else {
//                    mecanumCommand.pivot(0);
//                }
//            }

//            if (isOuttakeMotorOn) {
//                if (logitechsub.distance() >= 100) {
//                    outtakeCommand.setMaxRPM(3500);
//                } else if (logitechsub.distance() <= 90 && logitechsub.distance() >= 35) {
//                    outtakeCommand.setMaxRPM(2800);
//                } else if (logitechsub.distance() <= 35) {
//                    outtakeCommand.setMaxRPM(2300);
//                }
//            }


        }
    }
    public void processTelemetry() {
        telemetry.addData("RPM", hw.shooter.getVelocity() * 60.0 / 28.0);
        telemetry.addData("x", limelightsub.apriltag(telemetry));
        telemetry.addData("y", limelightsub.distance(telemetry));
        telemetry.addData("power", power);
        telemetry.addData("alliance", ALLIANCE);


        telemetry.update();
    }
}
