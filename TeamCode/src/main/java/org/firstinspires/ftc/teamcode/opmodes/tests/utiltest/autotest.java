package org.firstinspires.ftc.teamcode.opmodes.tests.utiltest;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumConstants;

@Config
@Autonomous(name = "Sample Auto Fixed")
public class autotest extends LinearOpMode {
    private MecanumCommand mecanumCommand;

    // Tunable PID constants via FTC Dashboard
    public static double KP_X = 0.03;
    public static double KP_Y = 0.03;
    public static double KP_THETA = 0.1;

    public static double TARGET_X = 0;
    public static double TARGET_Y = 20;
    public static double TARGET_THETA = 0;
    public static double VELOCITY = 30;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);

        // Set PID constants
        mecanumCommand.setConstants(
                KP_X, 0, 0,       // X-axis PID
                KP_Y, 0, 0,       // Y-axis PID
                KP_THETA, 0, 0    // Theta PID
        );

        telemetry.addLine("=== AUTONOMOUS READY ===");
        telemetry.addLine("Target: Move 20cm LEFT (Y+)");
        telemetry.addData("PID kpx", KP_X);
        telemetry.addData("PID kpy", KP_Y);
        telemetry.addData("PID kptheta", KP_THETA);
        telemetry.addLine();
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        // Reset odometry to start from (0, 0, 0)
        mecanumCommand.resetPinPointOdometry();
        sleep(100);

        // Set target position
        mecanumCommand.setFinalPosition(VELOCITY, TARGET_X, TARGET_Y, TARGET_THETA);

        ElapsedTime runtime = new ElapsedTime();
        int loopCount = 0;

        // Main control loop
        while (opModeIsActive() && mecanumCommand.positionNotReachedYet() && runtime.seconds() < 10.0) {
            // motorProcess() already calls processOdometry() internally
            mecanumCommand.motorProcess();

            // Telemetry for debugging
            telemetry.addData("Loop", loopCount++);
            telemetry.addData("Time", "%.2f sec", runtime.seconds());
            telemetry.addLine();

            telemetry.addLine("--- Current Position ---");
            telemetry.addData("X", "%.2f cm", mecanumCommand.getOdoX());
            telemetry.addData("Y", "%.2f cm", mecanumCommand.getOdoY());
            telemetry.addData("Heading", "%.2f°", Math.toDegrees(mecanumCommand.getOdoHeading()));
            telemetry.addLine();

            telemetry.addLine("--- Target ---");
            telemetry.addData("Target X", "%.2f", mecanumCommand.xFinal);
            telemetry.addData("Target Y", "%.2f", mecanumCommand.yFinal);
            telemetry.addData("Target Theta", "%.2f°", Math.toDegrees(mecanumCommand.thetaFinal));
            telemetry.addLine();

            telemetry.addLine("--- Errors ---");
            telemetry.addData("X Error", "%.2f cm", mecanumCommand.getXDifferencePinPoint());
            telemetry.addData("Y Error", "%.2f cm", mecanumCommand.getYDifferencePinPoint());
            telemetry.addData("Theta Error", "%.4f rad", mecanumCommand.getThetaDifferencePinPoint());
            telemetry.addLine();

            telemetry.addLine("--- Motor Powers ---");
            telemetry.addData("LF", "%.3f", hw.lf.getPower());
            telemetry.addData("RF", "%.3f", hw.rf.getPower());
            telemetry.addData("LB", "%.3f", hw.lb.getPower());
            telemetry.addData("RB", "%.3f", hw.rb.getPower());
            telemetry.addLine();

            telemetry.addData("X Reached?", mecanumCommand.isXReached() ? "YES" : "NO");
            telemetry.addData("Y Reached?", mecanumCommand.isYReached() ? "YES" : "NO");
            telemetry.addData("Theta Reached?", mecanumCommand.isThetaReached() ? "YES" : "NO");

            telemetry.update();

            sleep(20);  // Small delay for stability
        }

        mecanumCommand.stop();

        telemetry.addLine();
        telemetry.addLine("=== MOVEMENT COMPLETE ===");
        telemetry.addData("Final X", "%.2f cm", mecanumCommand.getOdoX());
        telemetry.addData("Final Y", "%.2f cm", mecanumCommand.getOdoY());
        telemetry.addData("Final Heading", "%.2f°", Math.toDegrees(mecanumCommand.getOdoHeading()));
        telemetry.addData("Runtime", "%.2f sec", runtime.seconds());
        telemetry.update();

        while (opModeIsActive()) {
            sleep(100);
        }
    }
}