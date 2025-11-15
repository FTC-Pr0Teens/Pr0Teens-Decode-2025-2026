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


@TeleOp(name = "TeleopSample", group = "TeleOp")
public class SampleTeleOpMode extends LinearOpMode {

    // opmodes should only own commands
    private MecanumCommand mecanumCommand;
    private OuttakeCommand outtakeCommand;
    private LimelightSubsystem limelightsub;
    private LogitechSubsystem logitechsub;
    private ElapsedTime timer;
    private Hardware hw;
    private ElapsedTime resetTimer;

    // --- Button Variables ---
    private boolean previousAState = false;
    private boolean previousBState = false;
    private boolean previousXState = false;
    private boolean previousYState = false;

    // --- Intake/Outake Variables---
    private boolean isIntakeMotorOn = false;
    private boolean isOuttakeMotorOn = false;

    // --- Pusher Variables ---
    private static final double PUSHER_UP = 0.2;
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

    private String ALLIANCE = "blue";

    @Override
    public void runOpMode() throws InterruptedException {
        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        outtakeCommand = new OuttakeCommand(hw);
        limelightsub = new LimelightSubsystem(hw, telemetry);

        hw.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        resetTimer = new ElapsedTime();
        hw.pusher.setPosition(PUSHER_DOWN);
        hw.sorter.setPosition(0);
        hw.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeInInit()){
            if (gamepad1.b){
                ALLIANCE = "red";
            }
            if(gamepad1.x){
                ALLIANCE = "blue";
            }
            telemetry.addData("Alliance: ",ALLIANCE);
            telemetry.update();
        }
        // Wait for start button to be pressed
        waitForStart();


        logitechsub = new LogitechSubsystem(hw, ALLIANCE);

        logitechsub.pattern();

        // Loop while OpMode is running
        while (opModeIsActive()) {

//            if (obelisk == "PPG"){
//
//            } else if (obelisk == "PGP"){
//
//            } else if (obelisk == "GPP"){
//
//            }

            mecanumCommand.processOdometry();
            mecanumCommand.fieldOrientedMove(
                    gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );

            processTelemetry();

            if (gamepad1.start){
                mecanumCommand.resetPinPointOdometry();
            }

            // --- Intake toggle on A (edge) ---
            boolean currentAState = gamepad1.a;
            if (currentAState && !previousAState) {
                isIntakeMotorOn = !isIntakeMotorOn;
                hw.intake.setPower(isIntakeMotorOn ? 0.8 : 0.0);
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

                if (isOuttakeMotorOn){
                   spunUp = outtakeCommand.spinup();
                } else if(!isOuttakeMotorOn){
                    outtakeCommand.stopShooter();
                }
            }
            previousXState = currentXState;

            boolean currentBState = gamepad1.b;
            if (currentBState && !previousBState) {
                limelightsub.ballPosition(telemetry, mecanumCommand);
            }
            previousBState = currentBState;

            if (gamepad1.b && sorterTimer.milliseconds() > 800 && !isPusherUp){
                sorterTimer.reset();
                if (sorterpos == 0) {
                    hw.sorter.setPosition(SORTER_FIRST_POS);//60 degrees
                }
                else if (sorterpos == 1) {
                    hw.sorter.setPosition(SORTER_SECOND_POS);//60 degrees
                }
                else if (sorterpos == 2) {
                    hw.sorter.setPosition(SORTER_THIRD_POS);//60 degrees
                }
                sorterpos = (sorterpos+1)%3;
            }

        }

    }
    public void processTelemetry(){
        //add telemetry messages here
        //telemetry.addData("resetTimer: ",  resetTimer.milliseconds());
        telemetry.addLine("---------------------------------");
        telemetry.addData("X", mecanumCommand.getX());
        telemetry.addData("Y", mecanumCommand.getY());
        telemetry.addData("Pusher ON", isPusherUp);
        telemetry.addData("Pattern ", obelisk);
        telemetry.addData("TPS: ", hw.shooter.getVelocity());
        telemetry.addData("RPM: ", hw.shooter.getVelocity() * 60.0 / 28.0);
        telemetry.update();
    }
}