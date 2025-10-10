package org.firstinspires.ftc.teamcode.opmodes.tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.pidcore.PIDCore;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;



@Config
@Autonomous (name = "Sample Auto test")
public class odoautotest extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private TelemetryPacket packet;
    private FtcDashboard dash;
    private int stage1 = 0;
    public double xFinal;
    public double yFinal;
    private double thetaFinal;
    private double velocity;



    @Override
    public void runOpMode() throws InterruptedException {
        // create Hardware using hardwareMap
        Hardware hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);

        enum AUTO_STATE {
            FIRST_BUCKET,
            SUB_PICKUP,
            FINISH

        }
        boolean firstInstance = true;
        dash = FtcDashboard.getInstance();
        telemetry = dash.getTelemetry();
        packet = new TelemetryPacket();
        ElapsedTime timer;


        AUTO_STATE autoState = AUTO_STATE.FIRST_BUCKET;
        waitForStart();
        while (opModeIsActive()) {
            // run processes
            updateTelemetry();

            mecanumCommand.motorProcess();
            mecanumCommand.processPIDUsingPinpoint();
            moveToPos(0, 15, 0);
            stopRobot();

        }
    }

    public void updateTelemetry() {
        telemetry.addData("x: ", mecanumCommand.getOdoX());
        telemetry.addData("y: ", mecanumCommand.getOdoY());
//        packet.put("theta: ", mecanumCommand.getOdoHeading());
    }
//
//    public void moveToPos(double x, double y, double theta) {
//        mecanumCommand.setFinalPositionMotionProfile(30, 30, x, y);
//        if (!mecanumCommand.positionNotReachedYet()) {
//            stage1++;
//        }
//    }
public void moveToPos(double x, double y, double theta) {
        setFinalPositionMotionProfile(true, 30, x, y, theta);


    }
    public void setFinalPositionMotionProfile(boolean run, double velocity, double x, double y, double theta){
        if (run){
            this.xFinal = x;
            this.yFinal = y;
            this.thetaFinal = theta;
            this.velocity = velocity;

        }
    }



        private void stopRobot() {
            mecanumCommand.stop();
        }


//    public void processPinPoint() {
//        pinPointOdo.deadReckoning();
//    }


    }


