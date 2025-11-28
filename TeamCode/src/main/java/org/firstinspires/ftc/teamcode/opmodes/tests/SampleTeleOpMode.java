package org.firstinspires.ftc.teamcode.opmodes.tests;

//import static org.firstinspires.ftc.teamcode.subsystems.cameras.LogitechSubsystem.obelisk;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.odometry.PinPointOdometrySubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.cameras.LimelightSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.cameras.LogitechSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.sorting.SortingSubsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


@TeleOp(name = "TeleopSample", group = "TeleOp")
public class SampleTeleOpMode extends LinearOpMode {
    enum pushme{
        UP,
        DOWN,
        TURN,

    }

    // opmodes should only own commands
    private MecanumCommand mecanumCommand;
    private OuttakeCommand outtakeCommand;
    private OuttakeSubsystem outtakeSubsystem;
    //    private LimelightSubsystem limelightsub;
    // private LogitechSubsystem logitechsub;
    // private SortingSubsystem sortingSubsystem;
    private ElapsedTime timer;
    private Hardware hw;
    private ElapsedTime resetTimer;
    private FtcDashboard dash;

    // --- Button Variables ---
    private boolean previousAState = false;
    private boolean previousBState = false;
    private boolean previousXState = false;
    private boolean previousYState = false;
    private boolean previousDownState = false;


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
    private static final double SORTER_SECOND_POS = 0.4;
    private static final double SORTER_THIRD_POS = 0.85;

    private TelemetryPacket packet;

    boolean spunUp;
    int counter = 1;
    private String ALLIANCE = "blue";
    private String motif;
    double stage = 0;
    double position = SORTER_FIRST_POS;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        outtakeCommand = new OuttakeCommand(hw);
        outtakeSubsystem = new OuttakeSubsystem(hw);
//        limelightsub = new LimelightSubsystem(hw, telemetry);
        // sortingSubsystem = new SortingSubsystem(hw, telemetry, motif);
        pushme autoState = pushme.UP;
        hw.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.pusher1.setDirection(Servo.Direction.REVERSE);
        resetTimer = new ElapsedTime();
        hw.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            hw.light.setPosition(0);
            telemetry.addData("Alliance: ", ALLIANCE);
            telemetry.update();
        }
        // Wait for start button to be pressed
        waitForStart();


        //logitechsub = new LogitechSubsystem(hw, ALLIANCE);

        // Loop while OpMode is running
        while (opModeIsActive()) {
            //logitechsub.pattern();
            // logitechsub.telemetryAprilTag(telemetry);

//            if (obelisk == "PPG"){
//
//            } else if (obelisk == "PGP"){
//
//            } else if (obelisk == "GPP"){
//
//            }

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

            boolean currentLBumpState = gamepad1.right_bumper;


            if (currentLBumpState && !previousLBumpState) {
                    hw.intake.setPower(isIntakeMotorOn ? -1.0 : 0.0);
                }

//                switch (autoState) {
//                    case UP:
//                        if(stage == 0){
//                            pusherTimer.reset();
//                            stage++;
//                        } else if (stage == 1 && pusherTimer.milliseconds() >= 300) {
//                            hw.pusher.setPosition(PUSHER_UP);
//                            hw.pusher1.setPosition(PUSHER_UP1);
//
//                            stage++;
//                            pusherTimer.reset();
//                            autoState = pushme.DOWN;
//                        }
//
//                    case DOWN:
//                        if ( stage == 2 && pusherTimer.milliseconds() >= 500) {
//                            hw.pusher.setPosition(PUSHER_DOWN);
//                            hw.pusher1.setPosition(PUSHER_DOWN1);
//                            pusherTimer.reset();
//                            stage++;
//                            autoState = pushme.TURN;
//                        }
//                    case TURN:
//                        if (stage == 3 && pusherTimer.milliseconds() >= 400) {
//                            if (position == SORTER_FIRST_POS){
//                                hw.sorter.setPosition(SORTER_SECOND_POS);
//                            }
//                            else if (position == SORTER_SECOND_POS){
//                                hw.sorter.setPosition(SORTER_THIRD_POS);
//                            }
//
//                        }     else {
//                            hw.sorter.setPosition(SORTER_FIRST_POS);
//                        }
//
//                }



            
            previousLBumpState = currentLBumpState;

            // --- Intake toggle on A ---
            boolean currentAState = gamepad1.a;
            if (currentAState && !previousAState) {
                hw.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                isIntakeMotorOn = !isIntakeMotorOn;
                hw.intake.setPower(isIntakeMotorOn ? 1.0 : 0.0);
            }
            previousAState = currentAState;

            // --- Pusher up on Y  ---
            boolean currentYState = gamepad1.y;
            if (currentYState && !previousYState) {
                // Start pulse only if not already pulsing
                if (!isPusherUp) {
                    hw.pusher.setPosition(0.39);
                    hw.pusher1.setPosition(0.19);
                    pusherTimer.reset();
                    isPusherUp = true;
                }
            }
            previousYState = currentYState;

            // Pusher down
            if (isPusherUp && pusherTimer.milliseconds() >= PUSHER_TIME) {
                hw.pusher.setPosition(0);
                hw.pusher1.setPosition(0);
                isPusherUp = false;
            }

            // Outtake
            boolean currentXState = gamepad1.dpad_up;


            if (currentXState && !previousXState) {
                isOuttakeMotorOn = !isOuttakeMotorOn;
            }
            previousXState = currentXState;

            if (isOuttakeMotorOn) {
//                hw.shooter.setVelocityPIDFCoefficients(40,0,4,0);
                outtakeCommand.spinup();

                lightOn(spunUp);
            } else if (!isOuttakeMotorOn) {
                hw.shooter.setVelocityPIDFCoefficients(40,0,0,0);
                outtakeCommand.stopShooter();
                hw.light.setPosition(0);
            }

            boolean currentDownState = gamepad1.dpad_down;
            if(currentDownState && previousDownState){
                isOuttakeMotorOn = !isOuttakeMotorOn;
            }
            previousDownState = currentDownState;
            if (isOuttakeMotorOn) {
                hw.shooter.setVelocityPIDFCoefficients(40,0,0,0);
                outtakeCommand.spinupmid();

                lightOn(spunUp);
            } else if (!isOuttakeMotorOn) {
                hw.shooter.setVelocityPIDFCoefficients(40,0,0,0);
                outtakeCommand.stopShooter();
                hw.light.setPosition(0);
            }



            boolean currentBState = gamepad1.b;
            if (currentBState && !previousBState) {
//                limelightsub.ballPosition(telemetry, mecanumCommand);
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
                //  sortingSubsystem.temporarySort();
            }
            dash.sendTelemetryPacket(packet);
            ;

        }


    }

    public void processTelemetry() {
        //add telemetry messages here
        //telemetry.addData("resetTimer: ",  resetTimer.milliseconds());
        telemetry.addLine("---------------------------------");
        telemetry.addData("X", mecanumCommand.getX());
        telemetry.addData("Y", mecanumCommand.getY());
        telemetry.addData("Pusher ON", isPusherUp);
        //telemetry.addData("Pattern ", logitechsub.pattern());
        telemetry.addData("TPS: ", hw.shooter.getVelocity());
        //telemetry.addData("Pattern ", obelisk);
        telemetry.addData("light:  ", spunUp);
        //telemetry.addData("x ", logitechsub.targetApril());
        telemetry.addData("heading:  ", mecanumCommand.getOdoHeading());
        telemetry.addData("ticks", hw.shooter.getVelocity());
        telemetry.addData("RPM",hw.shooter.getVelocity() * 60.0 / 28.0);
        packet.put("ticks:  ", hw.shooter.getVelocity());

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