package org.firstinspires.ftc.teamcode.opmodes.tests.utiltest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeSubsystem;

@Config
@TeleOp(name = "vel pid", group = "Test")
public class movmentcenter extends LinearOpMode {


    public static double Kp = 0.0;
    public static double Ki = 0.0;
    public static double Kd = 0.0;
    public static double targetRPM = 5067;

    private DcMotor shooterMotor;
    private OuttakeSubsystem outtakeSubsystem;
    private Hardware hw;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = Hardware.getInstance(hardwareMap);
        outtakeSubsystem = new OuttakeSubsystem(hw);
        shooterMotor = hw.shooter;

        FtcDashboard dashboard = FtcDashboard.getInstance();

        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Shooter PID Dashboard Ready");
        telemetry.addLine("Tune PID values in FTC Dashboard");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) targetRPM += 50;
            if (gamepad1.dpad_down) targetRPM -= 50;


            OuttakeSubsystem.Kp = Kp;
            OuttakeSubsystem.Ki = Ki;
            OuttakeSubsystem.Kd = Kd;

            double currentRPM = hw.shooter.getVelocity();
            double output = outtakeSubsystem.outputPositional(targetRPM, currentRPM);

            shooterMotor.setPower(output);

            // Telemetry
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current RPM", currentRPM);
            telemetry.addData("PID Output", output);
            telemetry.addData("Kp", Kp);
            telemetry.addData("Ki", Ki);
            telemetry.addData("Kd", Kd);
            telemetry.addData("Error", targetRPM - currentRPM);
            telemetry.update();
        }

        shooterMotor.setPower(0);
    }
}