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
        INTAKE_ONE,
        TURN_ONE,
        SUBMERSIBLE_PICKUP,
        PICKUP_FIRST,

    }
    AUTO_STATE autoState = AUTO_STATE.INTAKE_ONE;
    public static double kpx = 0.057;
    public static double kpy = 0.057;
    public static double kdx = 0.0073;
    public static double kdy = 0.0073;
    public static double kpTheta = 1.13;
    public static double kdTheta = 0.05;
    public static double kix = 0;
    public static double kiy = 0;
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
            mecanumCommand.processPIDUsingPinpoint();
            mecanumCommand.processOdometry();

            switch (autoState) {

                case INTAKE_ONE:
                    mecanumCommand.moveToPos(-100, 0, 0);// set target

                    if (!mecanumCommand.positionNotReachedYet()) {
                        autoState = AUTO_STATE.TURN_ONE;
                    }
                    break;

                case TURN_ONE:
                    mecanumCommand.moveToPos(mecanumCommand.getOdoX(), mecanumCommand.getOdoY(), 1.6);

                    if (!mecanumCommand.positionNotReachedYet()) {
                        sleep(200);
                        autoState = AUTO_STATE.SUBMERSIBLE_PICKUP;
                    }


                        break;


                case SUBMERSIBLE_PICKUP:
                    double targetY = -85;
                    double targetHeading = 1.62;
                    double tolerance = 0.5;


                    mecanumCommand.moveToPos(120, targetY, targetHeading);


                    if (Math.abs(mecanumCommand.getOdoY() - targetY) < tolerance) {
                        mecanumCommand.stop();
                        autoState = AUTO_STATE.PICKUP_FIRST;
                    }
                    break;


                case PICKUP_FIRST:
//                    mecanumCommand.moveToPos(0, 0, 0);
                    mecanumCommand.stop();
                    liftCommand.stopintake();
                    mecanumCommand.moveToPos(0, 0, 0);


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




