package org.firstinspires.ftc.teamcode.opmodes.tests.utiltest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.camera.LimeLightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;

@Config
@TeleOp(name = "Ball Tracking Test")
public class turrettest extends LinearOpMode {
    private Hardware hw;
    private MecanumCommand mecanumCommand;
    private LimeLightSubsystem limeLightSubsystem;
    private FtcDashboard dash;
    private TelemetryPacket packet;
    private Limelight3A limelight;

    // PID constants for mecanum drive
    public static double kpx = 0.05;
    public static double kpy = 0.1;
    public static double kdx = 0.0020;
    public static double kdy = 0.0020;
    public static double kpTheta = 1.6;
    public static double kdTheta = 0.035;
    public static double kix = 600;
    public static double kiy = 1100;
    public static double kitheta = 40000;

    // Ball tracking constants - tunable via FTC Dashboard
    public static double strafeGain = 0.03;      // Gain for horizontal centering (tx)
    public static double forwardGain = 0.02;     // Gain for approaching ball (ty)
    public static double turnGain = 0.01;        // Optional: turn while approaching
    public static double targetArea = 5.0;       // Stop when ball fills this % of screen
    public static double forwardBasePower = 0.2; // Base forward speed
    public static double maxSpeed = 0.8;         // Maximum motor power
    public static double minSpeed = 0.1;         // Minimum motor power threshold

    // Filtering for smoother movement
    private double filteredTx = 0;
    private double filteredTy = 0;
    private double filteredTa = 0;
    public static double filterAlpha = 0.6;      // 0 = no smoothing, 1 = no filtering

    @Override
    public void runOpMode() throws InterruptedException {
        hw = Hardware.getInstance(hardwareMap);
        dash = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);  // Ball detection pipeline
        mecanumCommand = new MecanumCommand(hw);
        limeLightSubsystem = new LimeLightSubsystem(hw);

        mecanumCommand.setConstants(kpx, kdx, kix,
                kpy, kdy, kiy,
                kpTheta, kdTheta, kitheta);
        telemetry.update();

        waitForStart();

        limelight.start();
        boolean trackingEnabled = true;

        while (opModeIsActive()) {

            mecanumCommand.motorProcess();
            mecanumCommand.processPIDUsingPinpoint();
            mecanumCommand.processOdometry();


            if (gamepad1.a) {
                trackingEnabled = !trackingEnabled;
                while (gamepad1.a && opModeIsActive()) {
                    Thread.sleep(10);
                }
                telemetry.addData("Tracking", trackingEnabled ? "ENABLED" : "DISABLED");
                telemetry.update();
            }


            LLResult result = limelight.getLatestResult();

            if (trackingEnabled && result != null && result.isValid()) {

                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();


                filteredTx = filterAlpha * tx + (1 - filterAlpha) * filteredTx;
                filteredTy = filterAlpha * ty + (1 - filterAlpha) * filteredTy;
                filteredTa = filterAlpha * ta + (1 - filterAlpha) * filteredTa;


                if (filteredTa > targetArea) {

                    mecanumCommand.moveToPos(0, 0, 0);
                    telemetry.addLine("✓ BALL REACHED - Stopping");
                    telemetry.addData("Ball Area", "%.2f%%", filteredTa);
                } else {

                    double strafe = -filteredTx * strafeGain;



                    double forward = forwardBasePower - (filteredTy * forwardGain);

                    double turn = -filteredTx * turnGain;

                    strafe = clampSpeed(strafe, maxSpeed, minSpeed);
                    forward = clampSpeed(forward, maxSpeed, minSpeed);
                    turn = clampSpeed(turn, maxSpeed * 0.5, 0);

                    mecanumCommand.moveToPos(forward, strafe, turn);


                    telemetry.addData("tx (horizontal)", "%.2f°", filteredTx);
                    telemetry.addData("ty (vertical)", "%.2f°", filteredTy);
                    telemetry.addData("ta (area)", "%.2f%%", filteredTa);
                    telemetry.addData("Commands", "F:%.2f S:%.2f T:%.2f", forward, strafe, turn);
                }

                telemetry.addData("Tracking", "ENABLED");
                telemetry.addData("Raw tx", "%.2f", tx);
                telemetry.addData("Raw ty", "%.2f", ty);
                telemetry.addData("Raw ta", "%.2f", ta);

            } else if (trackingEnabled) {
                mecanumCommand.moveToPos(0, 0, 0);

            } else {

                mecanumCommand.moveToPos(0, 0, 0);
                telemetry.addData("Tracking", "DISABLED");
                telemetry.addLine("Press A to enable tracking");
            }

            telemetry.addLine();
            telemetry.addData("Controls", "A: Toggle Tracking | Gamepad for manual");
            telemetry.update();

            // Dashboard telemetry
            packet = new TelemetryPacket();
            packet.put("Tracking", trackingEnabled);
            packet.put("Result Valid", result != null && result.isValid());
            if (result != null && result.isValid()) {
                packet.put("tx", result.getTx());
                packet.put("ty", result.getTy());
                packet.put("ta", result.getTa());
                packet.put("Filtered tx", filteredTx);
                packet.put("Filtered ty", filteredTy);
                packet.put("Filtered ta", filteredTa);
            }
            dash.sendTelemetryPacket(packet);
        }

        limelight.stop();
    }


    private double clampSpeed(double speed, double max, double min) {

        if (Math.abs(speed) < 0.01) {
            return 0;
        }

        double clampedSpeed = Math.max(-max, Math.min(speed, max));


        if (Math.abs(clampedSpeed) > 0 && Math.abs(clampedSpeed) < min) {
            clampedSpeed = Math.signum(clampedSpeed) * min;
        }

        return clampedSpeed;
    }
}