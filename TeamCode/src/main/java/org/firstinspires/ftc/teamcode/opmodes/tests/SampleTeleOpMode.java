package org.firstinspires.ftc.teamcode.opmodes.tests;

//import static org.firstinspires.ftc.teamcode.subsystems.cameras.LogitechSubsystem.obelisk;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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


@TeleOp(name = "Pr0teens test Teleop", group = "TeleOp")
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
    private static final double SORTER_SECOND_POS = 0.45;
    private static final double SORTER_THIRD_POS = 0.90;

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
        hw.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeCommand.setMaxRPM(2000);
        packet = new TelemetryPacket();

        while (opModeInInit()) {
            processTelemetry();



        }

    }

    public void processTelemetry() {
        telemetry.addData("Pattern ", logitechsub.pattern());
//        telemetry.addData("TPS: ", hw.shooter.getVelocity());
        //telemetry.addData("Pattern ", obelisk);
        telemetry.addData("color:  ", limelightsub.ballColor(telemetry));
//        telemetry.addData("light:  ", spunUp);
        telemetry.addData("x ", logitechsub.targetApril());
        telemetry.addData("y ", logitechsub.distance());
        telemetry.addData("x: ", mecanumCommand.getOdoX());
        telemetry.addData("y: ", mecanumCommand.getOdoY());
        telemetry.addData("heading:  ", mecanumCommand.getOdoHeading());
//        telemetry.addData("ticks", hw.shooter.getVelocity());
        telemetry.addData("RPM", hw.shooter.getVelocity() * 60.0 / 28.0);
//        packet.put("ticks:  ", hw.shooter.getVelocity());

        telemetry.update();
    }
}