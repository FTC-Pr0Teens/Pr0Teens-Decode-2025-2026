package org.firstinspires.ftc.teamcode.opmodes.tests.utiltest;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeConstants;
import org.firstinspires.ftc.teamcode.util.pidcore.PIDCore;


@TeleOp(name = "out")
public class outtest extends  LinearOpMode {
    private ElapsedTime timer;
    private DcMotorEx shooter;
    private OuttakeCommand outtakeCommand;
    private Hardware hw;
;


    public void runOpMode() throws InterruptedException {

        timer = new ElapsedTime();
        // Set motor directions
        FtcDashboard dash = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();


        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        hw = Hardware.getInstance(hardwareMap);
        outtakeCommand = new OuttakeCommand(hw);


        waitForStart();
        while (opModeIsActive()) {
            double curretVel = hw.shooter.getVelocity();
            double targetVel = 3000;
            if(gamepad1.a){
            hw.shooter.setPower(outputPositional(3000, curretVel));
            }
            packet.put("Target Velocity", targetVel);
            packet.put("Actual Velocity", curretVel);
            packet.put("Error", targetVel - curretVel);
            dash.sendTelemetryPacket(packet);

            telemetry.addData("Target Velocity", targetVel);
            telemetry.addData("Actual Velocity", curretVel);
            telemetry.update();


            }



    }

    public double outputPositional(double targetRPM, double curretRPM) {

         PIDCore pidCore;
         OuttakeConstants outtakeConstants;
         double derivative = 0;
         double error = 0;
         double lastError = 0;
         double integralSum = 0;
         double timeChange = 0;
         double errorChange = 0;

         double Kd = 0.002;
         double Kp = 0.03;
         double Ki = 0.00;
        // Output tracking
         double outputPositionalValue = 0;


        // Control flags
         boolean activateIntegral = false;

        // Safety limits
         double integralLimit = 1000.0; // Prevent integral windup
         double outputLimit = 1.0;      // Limit output magnitude
        // Ignore small errors

        // Time tracking
         double lastTime = 0;

        error = targetRPM - curretRPM;



        double currentTime = timer.seconds();
        double deltaTime = currentTime - lastTime;
        if (deltaTime <= 0) {
            deltaTime = 0.001;
        }


        derivative = (error - lastError) / deltaTime;


        if (activateIntegral && Math.abs(integralSum) < integralLimit) {
            integralSum += error * deltaTime;

            if (Math.abs(integralSum) > integralLimit) {
                integralSum = integralLimit * Math.signum(integralSum);
            }
        } else if (!activateIntegral) {
            integralSum = 0;
        }


        outputPositionalValue = (error * Kp) + (derivative * Kd) + (integralSum * Ki);

        if (Math.abs(outputPositionalValue) > outputLimit) {
            outputPositionalValue = outputLimit * Math.signum(outputPositionalValue);
        }


        lastError = error;
        lastTime = currentTime;
        errorChange = error - lastError;
        timeChange = deltaTime;

        return outputPositionalValue;
    }


            }


