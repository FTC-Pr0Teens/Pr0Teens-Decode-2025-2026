package org.firstinspires.ftc.teamcode.opmodes.tests.utiltest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp(name = "chaewon auto")
public class movmentcenter extends LinearOpMode {

    Limelight3A limelight;
    private TelemetryPacket packet;
    private FtcDashboard dash;
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;
    private ElapsedTime timer;


    private static final double KP_YAW = 0.02;
    private static final double KD_YAW = 0.006;

    private static final double KP_X = 0.02;
    private static final double KD_X = 0.004;

    private static final double KP_Y = 0.02;
    private static final double KD_Y = 0.004;


    private static final double MAX_TURN_POWER = 0.6;
    private static final double MAX_MOVE_POWER = 0.4;


    private static final double YAW_TOLERANCE = 2.0;
    private static final double X_TOLERANCE = 2.0;
    private static final double Y_TOLERANCE = 2.0;


    private double previousError = 0;
    private double integralSum = 0;
    private double prevYawError = 0, prevXError = 0, prevYError = 0;



    @Override


    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Hardware hw = new Hardware(hardwareMap);
        timer = new ElapsedTime();


        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftRear = hardwareMap.get(DcMotor.class, "lb");
        rightRear = hardwareMap.get(DcMotor.class, "rb");


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");


        limelight.pipelineSwitch(6);


        waitForStart();
        limelight.start();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double yawError = getYawError(result);
                double xError = getXError(result);
                double yError = getYError(result);

                double dt = timer.seconds();
                timer.reset();


                double yawPower = pidControl(yawError, prevYawError, KP_YAW, KD_YAW, dt, MAX_TURN_POWER);
                double strafePower = pidControl(xError, prevXError, KP_X, KD_X, dt, MAX_MOVE_POWER);
                double forwardPower = pidControl(yError, prevYError, KP_Y, KD_Y, dt, MAX_MOVE_POWER);

                prevYawError = yawError;
                prevXError = xError;
                prevYError = yError;

                if (Math.abs(yawError) < YAW_TOLERANCE) yawPower = 0;
                if (Math.abs(xError) < X_TOLERANCE) strafePower = 0;
                if (Math.abs(yError) < Y_TOLERANCE) forwardPower = 0;

                drive(forwardPower, strafePower, yawPower);

                telemetry.addData("Yaw Error", yawError);
                telemetry.addData("X Error", xError);
                telemetry.addData("Y Error", yError);
                telemetry.addData("Forward Power", forwardPower);
                telemetry.addData("Strafe Power", strafePower);
                telemetry.addData("Turn Power", yawPower);
                telemetry.update();

            } else {
                stopMotors();
            }
        }
    }

    private double getYawError(LLResult result) {
        return result.getFiducialResults().get(0).getTargetXDegrees();
    }

    private double getXError(LLResult result) {

        return result.getFiducialResults().get(0).getTargetXPixels();
    }

    private double getYError(LLResult result) {

        return result.getFiducialResults().get(0).getTargetYPixels();
    }

    private double pidControl(double error, double prevError, double kP, double kD, double dt, double maxPower) {
        double derivative = (error - prevError) / dt;
        double output = (kP * error) + (kD * derivative);
        return Math.max(-maxPower, Math.min(maxPower, output));
    }

    private void drive(double forward, double strafe, double rotate) {
        double lf = forward + strafe + rotate;
        double rf = forward - strafe - rotate;
        double lb = forward - strafe + rotate;
        double rb = forward + strafe - rotate;

        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftRear.setPower(lb);
        rightRear.setPower(rb);
    }

    private void stopMotors() {
        drive(0, 0, 0);
    }
}

