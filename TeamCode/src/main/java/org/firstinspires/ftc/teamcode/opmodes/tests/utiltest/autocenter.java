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

@TeleOp(name = "center")
public class autocenter extends LinearOpMode {

    Limelight3A limelight;
    private TelemetryPacket packet;
    private FtcDashboard dash;
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;
    private ElapsedTime timer;


    private static final double KP = 0.03;
    private static final double KI = 0.0;
    private static final double KD = 0.006;

    private static final double YAW_TOLERANCE = 2.0;
    private static final double MAX_TURN_POWER = 0.5;


    private double previousError = 0;
    private double integralSum = 0;


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
            double yawError = getYawValue();

            if (limelight.getLatestResult().isValid() && Math.abs(yawError) > YAW_TOLERANCE) {
                turnRobot(yawError);
            } else {

                stopMotors();
            }


            telemetry.addData("Target Valid", limelight.getLatestResult().isValid());
            telemetry.addData("Yaw Error", "%.2f degrees", yawError);
            telemetry.addData("Status", Math.abs(yawError) <= YAW_TOLERANCE ? "Aligned" : "Correcting");
            telemetry.update();
        }
    }

        private double getYawValue () {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                LLResultTypes.FiducialResult[] fiducialResults = result.getFiducialResults().toArray(new LLResultTypes.FiducialResult[0]);

                if (fiducialResults != null && fiducialResults.length > 0) {

                    return fiducialResults[0].getTargetXDegrees();
                }

            }
            return 0;
        }



        private void turnRobot ( double error){
            double deltaTime = timer.seconds();
            timer.reset();

            integralSum += error * deltaTime;
            double derivative = (error - previousError) / deltaTime;


            double turnPower = (KP * error) + (KI * integralSum) + (KD * derivative);


            turnPower = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, turnPower));


            leftFront.setPower(turnPower);
            leftRear.setPower(turnPower);
            rightFront.setPower(-turnPower);
            rightRear.setPower(-turnPower);

            previousError = error;
        }

        private void stopMotors () {
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);

            integralSum = 0;
            previousError = 0;
        }
    }

