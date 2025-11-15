package org.firstinspires.ftc.teamcode.opmodes.tests;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
//import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretSubsystem;


@Config
@Autonomous(name = "Bwedrtvybuhn")
public class Blue extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private TurretSubsystem turretSubsystem;
//    private IntakeCommand intakeCommand;
    private OuttakeSubsystem outtakeSubsystem;

    enum AUTO_STATE {
        MOVEPRELOAD,
        TURNPRELOAD,
        PRELOAD_ONE,
        PRELOAD_TWO,
        PRELOAD_THREE,
        PRELOAD_EMPTY,
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
    public static double kpx = 0.05;
    public static double kpy = 0.05;
    public static double kdx = 0.0083;
    public static double kdy = 0.0073;
    public static double kpTheta = 1.5;
    public static double kdTheta = 0.0;
    public static double kix = 0;
    public static double kiy = 0;
    public static double kitheta = 40000;
    int sorterpos = 0;
    private static final double SORTER_FIRST_POS = 0.0;
    private static final double SORTER_SECOND_POS = 0.42;
    private static final double SORTER_THIRD_POS = 0.88;
    double stage = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
//        intakeCommand = new IntakeCommand(hw);
        turretSubsystem = new TurretSubsystem(hw);
        outtakeSubsystem = new OuttakeSubsystem(hw);
        hw.sorter.setPosition(SORTER_FIRST_POS);
        hw.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanumCommand.setConstants(kpx, kdx, kix,
                kpy, kdy, kiy,
                kpTheta, kdTheta, kitheta);

        ElapsedTime timer = new ElapsedTime();
        boolean paused = false;
        boolean submersibleTargetSet = false;

        waitForStart();
        pusherTimer.reset();
        while (opModeIsActive()) {
            telemetry.addLine("for sydney wong");
            mecanumCommand.motorProcess();
            mecanumCommand.processPIDUsingPinpoint();
            mecanumCommand.processOdometry ();
            if(mecanumCommand.moveToPos(0 , -25, 0)){
                mecanumCommand.stop();
            }

//
//            switch (autoState) {
//                case MOVEPRELOAD:
//
//                    mecanumCommand.moveToPos(-178 , 0, 0);
//                    if (mecanumCommand.isPositionreached()) {
//                        mecanumCommand.resetPinPointOdometry();
//                        mecanumCommand.stop();
//
//                            autoState = AUTO_STATE.TURNPRELOAD;
//
//                    }
//                    break;
//                case TURNPRELOAD:
//               mecanumCommand.moveToPos(0, 0, -0.65);
//                    if (mecanumCommand.isPositionreached()) {
//
////                        autoState = AUTO_STATE.PRELOAD_ONE;
//
//                    }
//                    break;
//
//
//                case PRELOAD_ONE:
//                    hw.shooter.setPower(outtakeSubsystem.outputPositional(4200, hw.shooter.getVelocity()));
//                    hw.intake.setPower(0.7);
//
//                        if (stage == 0 && pusherTimer.milliseconds() >= 700) {
//                            hw.pusher.setPosition(PUSHER_UP);
//                            stage++;
//                            pusherTimer.reset();
//                        } else if (stage == 1 && pusherTimer.milliseconds() >= 1000) {
//                            hw.pusher.setPosition(PUSHER_DOWN);
//                            stage++;
//                            pusherTimer.reset();
//                        } else if (stage == 2 && pusherTimer.milliseconds() >= 2000) {
//                            hw.sorter.setPosition(SORTER_SECOND_POS);
//                            stage = 0;
//                            autoState = AUTO_STATE.PRELOAD_TWO;
//                            pusherTimer.reset();
//
//                    }
//                    break;
//
//
//                case PRELOAD_TWO:
//                    hw.shooter.setPower(outtakeSubsystem.outputPositional(4200, hw.shooter.getVelocity()));
//                    hw.intake.setPower(0.7);
//
//                        if (stage == 0 && pusherTimer.milliseconds() >= 700) {
//                            hw.pusher.setPosition(PUSHER_UP);
//                            stage++;
//                            pusherTimer.reset();
//                        } else if (stage == 1 && pusherTimer.milliseconds() >= 1000) {
//                            hw.pusher.setPosition(PUSHER_DOWN);
//                            stage++;
//                            pusherTimer.reset();
//                        } else if (stage == 2 && pusherTimer.milliseconds() >= 2000) {
//                            hw.sorter.setPosition(SORTER_THIRD_POS);
//                            stage = 0;
//                            autoState = AUTO_STATE.PRELOAD_THREE;
//                            pusherTimer.reset();
//                        }
//
//                    break;
//                case PRELOAD_THREE:
//                    hw.shooter.setPower(outtakeSubsystem.outputPositional(4200, hw.shooter.getVelocity()));
//                    hw.intake.setPower(0.7);
//
//
//                        if (stage == 0 && pusherTimer.milliseconds() >= 700) {
//                            hw.pusher.setPosition(PUSHER_UP);
//                            stage++;
//                            pusherTimer.reset();
//                        } else if (stage == 1 && pusherTimer.milliseconds() >= 1000) {
//                            hw.pusher.setPosition(PUSHER_DOWN);
//                            stage++;
//                            pusherTimer.reset();
//                        } else if (stage == 2 && pusherTimer.milliseconds() >= 2000) {
//                            hw.sorter.setPosition(SORTER_FIRST_POS);
//                            stage = 0;
//                            autoState = AUTO_STATE.PRELOAD_EMPTY;
//                            pusherTimer.reset();
//
//                    }
//                    break;
//                case PRELOAD_EMPTY:
//                    hw.intake.setPower(0.7);
//                    hw.shooter.setPower(outtakeSubsystem.outputPositional(4700, hw.shooter.getVelocity()));
//
//
//                        if (stage == 0 && pusherTimer.milliseconds() >= 700) {
//                            hw.pusher.setPosition(PUSHER_UP);
//                            stage++;
//                            pusherTimer.reset();
//                        } else if (stage == 1 && pusherTimer.milliseconds() >= 1000) {
//                            hw.pusher.setPosition(PUSHER_DOWN);
//                            stage++;
//                            pusherTimer.reset();
//                        } else if (stage == 2 && pusherTimer.milliseconds() >= 2000) {
//                            hw.sorter.setPosition(SORTER_FIRST_POS);
//                            stage = 0;
//                            autoState = AUTO_STATE.INTAKE_ONE;
//                            pusherTimer.reset();
//
//                    }
//                    break;
//                case INTAKE_ONE:
//                    hw.shooter.setPower(0.0);
////                    mecanumCommand.moveToPos(-100, 0, 0);// set target
////
////                    if (!mecanumCommand.isPositionreached()) {
////                        autoState = AUTO_STATE.TURN_ONE;
////                    }
//                    break;
//
//                case TURN_ONE:
////                    mecanumCommand.moveToPos(mecanumCommand.getOdoX(), mecanumCommand.getOdoY(), 1.6);
////
////                    if (!mecanumCommand.isPositionreached()) {
////                        sleep(200);
////                        autoState = AUTO_STATE.SUBMERSIBLE_PICKUP;
////                    }
//
//
//                    break;
//
//
//                case SUBMERSIBLE_PICKUP:
////                    double targetY = -85;
////                    double targetHeading = 1.62;
////                    double tolerance = 0.5;
////
////
////                    mecanumCommand.moveToPos(120, targetY, targetHeading);
////
////
////                    if (Math.abs(mecanumCommand.getOdoY() - targetY) < tolerance) {
////                        mecanumCommand.stop();
////                        autoState = AUTO_STATE.PICKUP_FIRST;
////                    }
//                    break;
//
//
//                case PICKUP_FIRST:
////                    mecanumCommand.moveToPos(0, 0, 0);
//                    mecanumCommand.stop();
//                    liftCommand.stopintake();
//                    mecanumCommand.moveToPos(0, 0, 0);
//
//
//                    break;
//                default:
//                    mecanumCommand.stop();
//                    break;
//

            //}
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



