package org.firstinspires.ftc.teamcode.opmodes.tests;

//import static org.firstinspires.ftc.teamcode.subsystems.cameras.LogitechSubsystem.obelisk;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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
import org.firstinspires.ftc.teamcode.subsystems.odometry.PinPointOdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.sorting.SortingSubsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


@TeleOp(name = "Pr0teens Teleop", group = "TeleOp")
public class SampleTeleOpMode extends LinearOpMode {

    // opmodes should only own commands
    private MecanumCommand mecanumCommand;
    private OuttakeCommand outtakeCommand;
    private OuttakeSubsystem outtakeSubsystem;
    private LimelightSubsystem limelightsub;
     private LogitechSubsystem logitechsub;
     private SortingSubsystem sortingSubsystem;
    private ElapsedTime timer;
    private Hardware hw;
    private ElapsedTime resetTimer;
    private FtcDashboard dash;

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
    private static final double PUSHER_UP = 0.39;
    private static final double PUSHER_DOWN = 0.0;
    private static final double PUSHER_UP1 = 0.19;
    private static final double PUSHER_DOWN1 = 0;
    private static final long PUSHER_TIME = 750;
    private final ElapsedTime pusherTimer = new ElapsedTime();

    private boolean isPusherUp = false;

    // --- Sorter Variables ---
    private final ElapsedTime sorterTimer = new ElapsedTime();
    int sorterpos = 0;
    private static final double SORTER_FIRST_POS = 0.0;
    private static final double SORTER_SECOND_POS = 0.45;
    private static final double SORTER_THIRD_POS = 0.88;

    private TelemetryPacket packet;

    boolean spunUp;
    int counter = 1;
    private String ALLIANCE = "blue";
    private String motif = "GPP";
    private double velocity;
    private String colour;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        outtakeCommand = new OuttakeCommand(hw);
        outtakeSubsystem = new OuttakeSubsystem(hw);
        limelightsub = new LimelightSubsystem(hw, telemetry);
        sortingSubsystem = new SortingSubsystem(hw, telemetry, motif);
        logitechsub = new LogitechSubsystem(hw, ALLIANCE);

        hw.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.pusher1.setDirection(Servo.Direction.REVERSE);
        resetTimer = new ElapsedTime();
        hw.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeCommand.setMaxRPM(6000);
        dash = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

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

            telemetry.addData("Alliance: ", ALLIANCE);
            telemetry.update();
        }
        // Wait for start button to be pressed
        waitForStart();

        // Loop while OpMode is running
        while (opModeIsActive()) {
            sortingSubsystem.setMotif(logitechsub.pattern());
            //logitechsub.pattern();
            // logitechsub.telemetryAprilTag(telemetry);

//            if (obelisk == "PPG"){
//
//            } else if (obelisk == "PGP"){
//
//            } else if (obelisk == "GPP"){
//
//            }
            hw.color.enableLed(true);

            mecanumCommand.processOdometry();
            mecanumCommand.normalMove(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            processTelemetry();

            if (gamepad1.start) {
                mecanumCommand.resetPinPointOdometry();
            }

            boolean currentLBumpState = gamepad2.left_bumper;
            if (currentLBumpState && !previousLBumpState) {
                hw.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                isIntakeMotorOn = !isIntakeMotorOn;
                hw.intake.setPower(isIntakeMotorOn ? 1.0 : 0.0);
            }
            previousLBumpState = currentLBumpState;

            // --- Intake toggle on A ---
            boolean currentAState = gamepad2.a;
            if (currentAState && !previousAState) {
                hw.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                isIntakeMotorOn = !isIntakeMotorOn;
                hw.intake.setPower(isIntakeMotorOn ? 0.6 : 0.0);
            }
            previousAState = currentAState;

            // --- Pusher up on Y  ---
            boolean currentYState = gamepad2.y;
            if (currentYState && !previousYState) {
                // Start pulse only if not already pulsing
                if (!isPusherUp) {
                    hw.pusher.setPosition(PUSHER_UP);
                    hw.pusher1.setPosition(PUSHER_UP1);
                    pusherTimer.reset();
                    isPusherUp = true;
                }
            }
            previousYState = currentYState;

            // Pusher down
            if (isPusherUp && pusherTimer.milliseconds() >= PUSHER_TIME) {
                hw.pusher.setPosition(PUSHER_DOWN);
                hw.pusher1.setPosition(PUSHER_DOWN1);
                isPusherUp = false;
            }

            // Outtake
            boolean currentXState = gamepad2.x;

            if (currentXState && !previousXState) {
                isOuttakeMotorOn = !isOuttakeMotorOn;
            }
            previousXState = currentXState;

            if (isOuttakeMotorOn) {
                if (logitechsub.distance() >= 100) {
                    outtakeCommand.setMaxRPM(3900);
                    if ((hw.shooter.getVelocity() * 60.0 / 28.0) > 3800) {
                        if ((hw.shooter.getVelocity() * 60.0 / 28.0) < 4000) {
                            hw.light.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                        }
                    } else {
                        hw.light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    }
                }
                else if (logitechsub.distance() <= 90 && logitechsub.distance() >= 35) {
                    outtakeCommand.setMaxRPM(2900);
                    if ((hw.shooter.getVelocity() * 60.0 / 28.0) > 2800) {
                        if ((hw.shooter.getVelocity() * 60.0 / 28.0) < 3000) {
                            hw.light.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                        }
                    } else {
                        hw.light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    }
                }
                else if (logitechsub.distance() <= 35) {
                    outtakeCommand.setMaxRPM(2200);
                    if ((hw.shooter.getVelocity() * 60.0 / 28.0) > 2100) {
                        if ((hw.shooter.getVelocity() * 60.0 / 28.0) < 2300) {
                            hw.light.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                        }
                    } else {
                        hw.light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    }
                }

                if (logitechsub.targetApril() > 5) {
                    mecanumCommand.pivot(0.2);
                } else if (logitechsub.targetApril() < -5){
                    mecanumCommand.pivot(-0.2);
                } else {
                    mecanumCommand.pivot(0);
                }

                outtakeCommand.spinup();
//                lightOn(spunUp);
            } else if (!isOuttakeMotorOn) {
                outtakeCommand.stopShooter();
//                hw.light.setPosition(0);
            }

            // Sorter
            boolean currentBState = gamepad1.b;
            if (currentBState && !previousBState) {
//                limelightsub.ballPosition(telemetry, mecanumCommand);
            }
            previousBState = currentBState;

            if (gamepad2.b && sorterTimer.milliseconds() > 800 && !isPusherUp) {
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

//            if (gamepad1.right_bumper && sorterTimer.milliseconds() >= 500) {
//                telemetry.addLine("running Sort code");
//                sorterTimer.reset();
//                limelightsub.ballColor(telemetry);
//                //  sortingSubsystem.temporarySort();
//            }
//            dash.sendTelemetryPacket(packet);

//            if (isIntakeMotorOn == true && colour != null) {
//                sortingSubsystem.intake(colour);
//                limelightsub.telemetryLimelight(telemetry);
//            }

            if (gamepad2.b && sorterTimer.milliseconds() > 500) {
                colour = limelightsub.ballColor(telemetry);
                if (colour != null) {
                    sortingSubsystem.intake(colour);
                }
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
        //telemetry.addData("Pattern ", obelisk);
        telemetry.addData("color:  ", limelightsub.ballColor(telemetry));
        telemetry.addData("light:  ", spunUp);
        telemetry.addData("x ", logitechsub.targetApril());
        telemetry.addData("y ", logitechsub.distance());
        telemetry.addData("heading:  ", mecanumCommand.getOdoHeading());
        telemetry.addData("ticks", hw.shooter.getVelocity());
        telemetry.addData("RPM",hw.shooter.getVelocity() * 60.0 / 28.0);
        packet.put("ticks:  ", hw.shooter.getVelocity());

        telemetry.update();
    }

//    public void lightOn(boolean rpmReached) {
//        if (rpmReached && isOuttakeMotorOn) {
//            hw.light.setPosition(0.5);
//        } else {
//            hw.light.setPosition(0);
//        }
//
//
//    }
}