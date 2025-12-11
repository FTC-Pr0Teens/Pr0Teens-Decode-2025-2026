package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

    private MecanumCommand mecanumCommand;
    private OuttakeCommand outtakeCommand;

    private Hardware hw;

    // --- Button Variables ---
    private boolean previousAState = false;
    private boolean previousBState = false;
    private boolean previousXState = false;
    private boolean lastYState = false;
    private boolean previousLBumpState = false;
    private boolean isIntakeMotorOn = false;
    private boolean isOuttakeMotorOn = false;

    private static final double PUSHER_UP = 0.05;
    private static final double PUSHER_DOWN = 0.0;
    private static final double PUSHER_UP1 = 0.05;
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
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        mecanumCommand = new MecanumCommand(hw);
        outtakeCommand = new OuttakeCommand(hw);
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
//            hw.pusher.setPosition(PUSHER_UP);
//            hw.pusher1.setPosition(PUSHER_UP1);
            hw.stopper.setDirection(Servo.Direction.FORWARD);
            hw.stopper.setPosition(0.2);
//            hw.light.setPosition(0);

            telemetry.addData("alliance", ALLIANCE);
            telemetry.update();
        }
        limelightsub = new LimelightSubsystem(hw, telemetry, ALLIANCE);

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

            hw.light.setPosition(0.28);

            if (gamepad1.right_bumper) {
                hw.turret.setPower(-1.0);
            } else if (isOuttakeMotorOn) {
                outtakeCommand.setMaxRPM(outtakeCommand.getShooterRPM(limelightsub.distance(telemetry)));

//                outtakeCommand.setMaxRPM(3000);
                if (outtakeCommand.isRPMReached() == true) {
                    hw.light.setPosition(0.5);
                } else {
                    hw.light.setPosition(0.28);
                }

                outtakeCommand.spinup();
                hw.stopper.setDirection(Servo.Direction.FORWARD);
                hw.stopper.setPosition(0.5);

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

            packet.put("RPM", hw.shooter.getVelocity() * 60.0 / 28.0);
            dashboard.sendTelemetryPacket(packet);

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
