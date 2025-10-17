package org.firstinspires.ftc.teamcode.opmodes.tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommand;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumConstants;


@Config
@Autonomous (name = "Sample Auto")
public class SampleAutoOpMode extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private LiftCommand liftCommand;
    enum AUTO_STATE {
        HANG_ONE,
        PICKUP_ZERO,
        PICKUP_FIRST,

    }
    AUTO_STATE autoState = AUTO_STATE.HANG_ONE;
    public static double kpx = 0.0267;
    public static double kpy = 0.0267;
    public static double kdx = 0.00167;
    public static double kdy = 0.00167;
    public static double kpTheta = 1.3;
    public static double kdTheta = 0.056;
    public static double kix = 650;
    public static double kiy = 500;
    public static double kitheta = 40000;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        liftCommand = new LiftCommand(hw);

        mecanumCommand.setConstants(kpx, kdx, kix,
                kpy, kdy, kiy,
                kpTheta, kdTheta, kitheta);

        ElapsedTime timer = new ElapsedTime();
        boolean paused = false;
        boolean submersibleTargetSet = false;

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine("for sydney wong");
            mecanumCommand.motorProcess();
            mecanumCommand.processOdometry();
            mecanumCommand.processPIDUsingPinpoint();
            switch (autoState) {

                case HANG_ONE:
                    mecanumCommand.moveToPos(50, -50, Math.PI);
                    liftCommand.turret();
                    if (!mecanumCommand.positionNotReachedYet()) {
                        autoState = AUTO_STATE.PICKUP_ZERO;
                    }
                    break;
                case PICKUP_ZERO:
                    mecanumCommand.stop();
                    sleep(1000);
                    liftCommand.turretstop();
                    autoState = AUTO_STATE.PICKUP_FIRST;
                    break;

//                    case SUBMERSIBLE_PICKUP:
//                    if (!submersibleTargetSet) {  // flag variable
//                        kpx = 0.05; kpy = 0.1;
//                        kdx = 0.0017; kdy = 0.0017;
//                        kix = 650; kiy = 1100; kitheta = 40000;
//                        kpTheta = 1.6; kdTheta = 0.035;
//
//                        submersibleTargetSet = true;
//                    }
//
//                    if (!mecanumCommand.positionNotReachedYet()) {
//                        autoState = AUTO_STATE.PICKUP_FIRST;
//                    }
//                    break;

                case PICKUP_FIRST:

                    mecanumCommand.stop();

                    break;
                default:
                    mecanumCommand.stop();
                    break;


            }
            updateTelemetry();
        }
    }



    public void updateTelemetry() {
        telemetry.addData("x: ", mecanumCommand.getOdoX());
        telemetry.addData("y: ", mecanumCommand.getOdoY());
        telemetry.addData("Theta: ", mecanumCommand.getOdoHeading());




        telemetry.update();
    }


    }




