package org.firstinspires.ftc.teamcode.opmodes.tests.utiltest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.util.pidcore.PIDCore;

@TeleOp(name = "move center")
    public class movmentcenter extends LinearOpMode {


        Limelight3A limelight;
        private TelemetryPacket packet;
        private FtcDashboard dash;
        private ElapsedTime timer;
        private MecanumCommand mecanumCommand;
        private PIDCore pidCore;






        @Override


        public void runOpMode() throws InterruptedException {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            Hardware hw = new Hardware(hardwareMap);
            mecanumCommand = new MecanumCommand(hw);
            timer = new ElapsedTime();


            limelight = hardwareMap.get(Limelight3A.class, "limelight");


            limelight.pipelineSwitch(6);


            waitForStart();
            limelight.start();

            while (opModeIsActive()) {



                LLResult result = limelight.getLatestResult();


                if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {

                    double xError = result.getFiducialResults().get(0).getTargetXPixels();
                    double yawError = result.getFiducialResults().get(0).getTargetXPixels();
                    double yError = result.getFiducialResults().get(0).getTargetYPixels();
                    mecanumCommand.apriltrack(xError, yError,yawError);
                    telemetry.addData("Yaw Error", yawError);
                    telemetry.addData("X Error", xError);
                    telemetry.addData("Y Error", yError);

                } else {
                    telemetry.addLine("No fiducials detected");
                }
                telemetry.update();

            }
        }





}




