package org.firstinspires.ftc.teamcode.opmodes.tests.utiltest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.camera.LimeLightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.camera.Vec2;
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommand;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumConstants;

@TeleOp(name = "turret test")
public class turrettest extends LinearOpMode {
    private Limelight3A limelight;
    private MecanumCommand mecanumCommand;
    private LimeLightSubsystem LimeLightSubsystem;
    private FtcDashboard dash;
    private Vec2 samplePos;
    private TelemetryPacket packet;
    private ElapsedTime timer;
    private ElapsedTime profileTimer;
    private Hardware hw;
    private double angle;
    private int stage1 = 0;
    private Vec2 samplePos2;

    private double xTarget = 0;
    private double yTarget = 0;

    public static double kpx = 0.05;
    public static double kpy = 0.1;
    public static double kdx = 0.0020;
    public static double kdy = 0.0020;
    public static double kpTheta = 1.6;
    public static double kdTheta = 0.035;
    public static double kix = 600;
    public static double kiy = 1100;
    public static double kitheta = 40000;


    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hw = Hardware.getInstance(hardwareMap);
        dash = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        samplePos = new Vec2(0, 0);
        profileTimer = new ElapsedTime();
        timer = new ElapsedTime();
        limelight = hardwareMap.get(Limelight3A.class, "lime");
        limelight.pipelineSwitch(1);
        mecanumCommand = new MecanumCommand(hw);
        LimeLightSubsystem = new LimeLightSubsystem(hw);
        mecanumCommand.setConstants(kpx, kdx, kix,
                kpy, kdy, kiy,
                kpTheta, kdTheta, kitheta);

        waitForStart();

        limelight.start();


        while (opModeIsActive()) {
            mecanumCommand.motorProcess();
            mecanumCommand.processPIDUsingPinpoint();
            mecanumCommand.processOdometry();

            LLResult result = limelight.getLatestResult();
            Vec2 ballPos = LimeLightSubsystem.getBallPosition(result);

            if (ballPos != null) {
                double xOffset = ballPos.x; // horizontal offset (pixels or normalized)
                double yOffset = ballPos.y; // vertical offset (distance proxy)

                // Tune this threshold based on Limelight output scale — lower = closer
                double closeThreshold = 5;

                double strafe = -xOffset * 0.05;
                double forward = -yOffset * 0.05;


                if (Math.abs(yOffset) > closeThreshold) {
                    mecanumCommand.moveToPos(forward, strafe, 0);
                    telemetry.addData("Moving Toward Ball", true);
                } else {
                    mecanumCommand.moveToPos(0, 0, 0);
                    telemetry.addData("Moving Toward Ball", false);
                    telemetry.addLine("Ball reached — stopping.");
                }

                telemetry.addData("Tracking", true);
                telemetry.addData("Ball X", xOffset);
                telemetry.addData("Ball Y", yOffset);
            } else {
                mecanumCommand.moveToPos(0, 0, 0);
                telemetry.addData("Tracking", false);
            }

            telemetry.update();
        }
    }
}