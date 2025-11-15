package org.firstinspires.ftc.teamcode.opmodes.tests;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
//import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretSubsystem;


@Config
@Autonomous(name = "Red Auto")
public class RedFieldAuto extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private TurretSubsystem turretSubsystem;
//    private IntakeCommand intakeCommand;

    enum AUTO_STATE {
        MOVEPRELOAD,
        PRELOAD_ONE,
        PRELOAD_TWO,
        PRELOAD_THREE,
        INTAKE_ONE,
        TURN_ONE,
        SUBMERSIBLE_PICKUP,
        PICKUP_FIRST,

    }

    AUTO_STATE autoState = AUTO_STATE.MOVEPRELOAD;
    private static final double PUSHER_UP = 0.2;
    private static final double PUSHER_DOWN = 0;
    private static final long PUSHER_TIME = 500;
    private final ElapsedTime pusherTimer = new ElapsedTime();

    private boolean isPusherUp = false;
    public static double kpx = 0.055;
    public static double kpy = 0.057;
    public static double kdx = 0.0083;
    public static double kdy = 0.0073;
    public static double kpTheta = 0.8;
    public static double kdTheta = 0.08;
    public static double kix = 0;
    public static double kiy = 0;
    public static double kitheta = 40000;
    int sorterpos = 0;
    private static final double SORTER_FIRST_POS = 0.0;
    private static final double SORTER_SECOND_POS = 0.42;
    private static final double SORTER_THIRD_POS = 0.88;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
//        intakeCommand = new IntakeCommand(hw);
        turretSubsystem = new TurretSubsystem(hw);
        hw.sorter.setPosition(0);
        hw.shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
                case MOVEPRELOAD:
                    mecanumCommand.moveToPos(-39.43 , -2, 0.39);


                    if (mecanumCommand.isPositionReached()) {
                        autoState = AUTO_STATE.PRELOAD_ONE;

                    }


                case PRELOAD_ONE:

//                    hw.shooter.setPower(1.0);
                    hw.intake.setPower(0.7);
                    if (!isPusherUp && pusherTimer.milliseconds() == 0) {
                        pusherTimer.reset();
                    }
                    if (!isPusherUp && pusherTimer.milliseconds() >= 1500) {
                        hw.pusher.setPosition(PUSHER_UP);
                        pusherTimer.reset();
                        isPusherUp = true;
                    } else if (isPusherUp && pusherTimer.milliseconds() >= PUSHER_TIME) {
                        hw.pusher.setPosition(PUSHER_DOWN);
                        pusherTimer.reset();
                        isPusherUp = false;
                        sorterpos = 1;

                    }   else if (!isPusherUp && sorterpos == 0 && pusherTimer.milliseconds() >= 1000) {
                        hw.sorter.setPosition(SORTER_SECOND_POS);
                        autoState = AUTO_STATE.PRELOAD_TWO;
                    }

                    break;


                case PRELOAD_TWO:
//                    hw.shooter.setPower(1.0);
                    hw.intake.setPower(0.7);

                    if (!isPusherUp && pusherTimer.milliseconds() == 0) {
                        pusherTimer.reset();
                    }

                    if (!isPusherUp && pusherTimer.milliseconds() >= 1500) {
                        hw.pusher.setPosition(PUSHER_UP);
                        pusherTimer.reset();
                        isPusherUp = true;
                    } else if (isPusherUp && pusherTimer.milliseconds() >= PUSHER_TIME) {
                        hw.pusher.setPosition(PUSHER_DOWN);
                        pusherTimer.reset();
                        isPusherUp = false;
                        sorterpos = 1;
                    }     else if (!isPusherUp && sorterpos == 1 && pusherTimer.milliseconds() >= 1000) {
                        hw.sorter.setPosition(SORTER_THIRD_POS);
                        autoState = AUTO_STATE.PRELOAD_THREE;

                    }
                    break;
                case PRELOAD_THREE:
//                    hw.shooter.setPower(0.95);
                    hw.intake.setPower(0.7);

                    if (!isPusherUp && pusherTimer.milliseconds() == 0) {
                        pusherTimer.reset();
                    }

                    if (!isPusherUp && pusherTimer.milliseconds() >= 1500) {
                        hw.pusher.setPosition(PUSHER_UP);
                        pusherTimer.reset();
                        isPusherUp = true;
                    } else if (isPusherUp && pusherTimer.milliseconds() >= PUSHER_TIME) {
                        hw.pusher.setPosition(PUSHER_DOWN);
                        pusherTimer.reset();
                        isPusherUp = false;
                        sorterpos = 2;
                    }     else if (!isPusherUp && sorterpos == 2  && pusherTimer.milliseconds() >= 1000) {
                        hw.sorter.setPosition(SORTER_FIRST_POS);
                        autoState = AUTO_STATE.INTAKE_ONE;
                    }
                    break;

                case INTAKE_ONE:
                    hw.shooter.setPower(0.0);
//                    mecanumCommand.moveToPos(-100, 0, 0);// set target
//
//                    if (!mecanumCommand.isPositionreached()) {
//                        autoState = AUTO_STATE.TURN_ONE;
//                    }
                    break;

                case TURN_ONE:
//                    mecanumCommand.moveToPos(mecanumCommand.getOdoX(), mecanumCommand.getOdoY(), 1.6);
//
//                    if (!mecanumCommand.isPositionreached()) {
//                        sleep(200);
//                        autoState = AUTO_STATE.SUBMERSIBLE_PICKUP;
//                    }


                    break;


                case SUBMERSIBLE_PICKUP:
//                    double targetY = -85;
//                    double targetHeading = 1.62;
//                    double tolerance = 0.5;
//
//
//                    mecanumCommand.moveToPos(120, targetY, targetHeading);
//
//
//                    if (Math.abs(mecanumCommand.getOdoY() - targetY) < tolerance) {
//                        mecanumCommand.stop();
//                        autoState = AUTO_STATE.PICKUP_FIRST;
//                    }
                    break;


                case PICKUP_FIRST:
//                    mecanumCommand.moveToPos(0, 0, 0);
                    mecanumCommand.stop();
//                    intakeCommand.stopintake();
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



