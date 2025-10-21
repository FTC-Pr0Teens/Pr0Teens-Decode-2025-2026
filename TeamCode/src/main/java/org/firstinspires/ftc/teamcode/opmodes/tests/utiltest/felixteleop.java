package org.firstinspires.ftc.teamcode.opmodes.tests.utiltest;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommand;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;

@Autonomous(name="shoot test", group="Vision")
public class felixteleop extends LinearOpMode {

    private Limelight3A limelight;
    private OuttakeSubsystem outtakeSubsystem;

    private GoBildaPinpointDriver odo;

    // Current pose estimates (in cm or degrees as appropriate)
    private MecanumCommand mecanumCommand;
    private LiftCommand liftCommand;

    private ElapsedTime timer;

    private ElapsedTime resetTimer;

    private Hardware hw;
    private final double limelightHeight = 10;
    private final double targetHeight = 30;
    private final double limelightAngle = 20;

    @Override
    public void runOpMode() {
        outtakeSubsystem = new OuttakeSubsystem(hw);
        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        liftCommand = new LiftCommand(hw);

        limelight.pipelineSwitch(6);
        waitForStart();

        limelight.start();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy();

                double distance = outtakeSubsystem.getDistance(
                        limelightHeight,
                        targetHeight,
                        limelightAngle,
                        ty
                );


                double currentRPM = hw.shooter.getVelocity();
                double deltaTime = 0.02;
                outtakeSubsystem.aimAndShoot(hw.shooter, distance, currentRPM, deltaTime);

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Distance", "%.2f in", distance);
                telemetry.addData("Shooter RPM", "%.2f", currentRPM);

            } else {
                hw.shooter.setPower(0);
                telemetry.addData("Limelight", "No Targets");
            }

            telemetry.update();
        }
    }
}