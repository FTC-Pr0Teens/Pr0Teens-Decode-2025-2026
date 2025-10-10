package org.firstinspires.ftc.teamcode.opmodes.tests.utiltest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;

@Config
@Autonomous(name = "Debug Auto")
public class movetest extends LinearOpMode {
    private MecanumCommand mecanumCommand;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);

        telemetry.addLine("Initialized. Press START");
        telemetry.addLine("Robot will attempt to move to Y=20");
        telemetry.update();

        waitForStart();

        // TEST 1: Check if odometry is reading
        telemetry.addLine("=== TEST 1: Odometry Check ===");
        telemetry.update();
        sleep(1000);

        for (int i = 0; i < 10; i++) {
            mecanumCommand.processOdometry();
            telemetry.addData("Loop", i);
            telemetry.addData("Odo X", mecanumCommand.getOdoX());
            telemetry.addData("Odo Y", mecanumCommand.getOdoY());
            telemetry.addData("Odo Heading", Math.toDegrees(mecanumCommand.getOdoHeading()));
            telemetry.update();
            sleep(100);
        }

        sleep(2000);

        // TEST 2: Manual motor test - all motors forward
        telemetry.addLine("=== TEST 2: Motors Forward ===");
        telemetry.update();
        sleep(1000);

        hw.lf.setPower(0.3);
        hw.rf.setPower(0.3);
        hw.lb.setPower(0.3);
        hw.rb.setPower(0.3);
        sleep(1000);

        hw.lf.setPower(0);
        hw.rf.setPower(0);
        hw.lb.setPower(0);
        hw.rb.setPower(0);
        sleep(1000);

        // Check if odometry detected movement
        mecanumCommand.processOdometry();
        telemetry.addLine("After moving forward:");
        telemetry.addData("X", mecanumCommand.getOdoX());
        telemetry.addData("Y", mecanumCommand.getOdoY());
        telemetry.update();
        sleep(2000);

        // Reset position
        mecanumCommand.resetPinPointOdometry();
        sleep(500);

        // TEST 3: PID-based movement
        telemetry.addLine("=== TEST 3: PID Movement ===");
        telemetry.addLine("Target: X=0, Y=20, Theta=0");
        telemetry.update();
        sleep(1000);

        mecanumCommand.setFinalPosition(30, 0, 20, 0);

        ElapsedTime runtime = new ElapsedTime();
        int loopCount = 0;

        while (opModeIsActive() && runtime.seconds() < 5.0) {
            mecanumCommand.processOdometry();

            // Get position BEFORE motorProcess
            double currentX = mecanumCommand.getOdoX();
            double currentY = mecanumCommand.getOdoY();
            double currentHeading = mecanumCommand.getOdoHeading();

            mecanumCommand.motorProcess();

            // Get motor powers AFTER motorProcess
            double lfPower = hw.lf.getPower();
            double rfPower = hw.rf.getPower();
            double lbPower = hw.lb.getPower();
            double rbPower = hw.rb.getPower();

            // Detailed telemetry
            telemetry.addData("Loop", loopCount++);
            telemetry.addData("Time", "%.2f", runtime.seconds());
            telemetry.addLine();

            telemetry.addLine("--- Position ---");
            telemetry.addData("Current X", "%.2f", currentX);
            telemetry.addData("Current Y", "%.2f", currentY);
            telemetry.addData("Current Heading", "%.2f°", Math.toDegrees(currentHeading));
            telemetry.addLine();

            telemetry.addLine("--- Target ---");
            telemetry.addData("Target X", mecanumCommand.xFinal);
            telemetry.addData("Target Y", mecanumCommand.yFinal);
            telemetry.addData("Target Heading", "%.2f°", Math.toDegrees(mecanumCommand.thetaFinal));
            telemetry.addLine();

            telemetry.addLine("--- Errors ---");
            telemetry.addData("X Error", "%.2f", mecanumCommand.getXDifferencePinPoint());
            telemetry.addData("Y Error", "%.2f", mecanumCommand.getYDifferencePinPoint());
            telemetry.addData("Theta Error", "%.4f", mecanumCommand.getThetaDifferencePinPoint());
            telemetry.addLine();

            telemetry.addLine("--- Motor Powers ---");
            telemetry.addData("LF", "%.3f", lfPower);
            telemetry.addData("RF", "%.3f", rfPower);
            telemetry.addData("LB", "%.3f", lbPower);
            telemetry.addData("RB", "%.3f", rbPower);
            telemetry.addLine();

            telemetry.addLine("--- Status ---");
            telemetry.addData("X Reached?", mecanumCommand.isXReached());
            telemetry.addData("Y Reached?", mecanumCommand.isYReached());
            telemetry.addData("Theta Reached?", mecanumCommand.isThetaReached());

            telemetry.update();

            // Break if reached (with safety timeout)
            if (mecanumCommand.isXReached() && mecanumCommand.isYReached() && mecanumCommand.isThetaReached()) {
                break;
            }

            sleep(20);
        }

        mecanumCommand.stop();

        telemetry.addLine();
        telemetry.addLine("=== FINAL RESULTS ===");
        telemetry.addData("Final X", mecanumCommand.getOdoX());
        telemetry.addData("Final Y", mecanumCommand.getOdoY());
        telemetry.addData("Runtime", "%.2f sec", runtime.seconds());
        telemetry.update();

        while (opModeIsActive()) {
            sleep(100);
        }
    }
}
