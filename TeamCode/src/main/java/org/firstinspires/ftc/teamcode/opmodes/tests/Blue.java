package org.firstinspires.ftc.teamcode.opmodes.tests;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
//import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.cameras.LogitechSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretSubsystem;


@Config
@Autonomous(name = "Bwedrtvybuhn")
public class Blue extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private TurretSubsystem turretSubsystem;
    //    private IntakeCommand intakeCommand;
    private LogitechSubsystem logitechSubsystem;
    private OuttakeCommand outtakeCommand;

    enum AUTO_STATE {
        MOVEPRELOAD,
        TURNPRELOAD,
        PRELOAD_ONE,
        PRELOAD_TWO,
        PRELOAD_THREE,
        PRELOAD_EMPTY,
        INTAKE_ONE,
        INTAKE_MOVE,
        INTAKE_TWO,
        INTAKE_THREE,
        INTAKE_SHOOT,
        INTAKE_SHOOT1,
        INTAKE_SHOOT2,
        CLEAR,
        NINEBALL_1,
        NINEBALL_2,
        NINEBALL_3,
        TURN_ONE,
        SUBMERSIBLE_PICKUP,
        LEAVE,
        PICKUP_FIRST,

    }

    AUTO_STATE autoState = AUTO_STATE.MOVEPRELOAD;
    private static final double PUSHER_UP = 0.39;
    private static final double PUSHER_DOWN = 0.0;
    private static final double PUSHER_UP1 = 0.19;
    private static final double PUSHER_DOWN1 = 0;
    private static final long PUSHER_TIME = 300;
    private final ElapsedTime pusherTimer = new ElapsedTime();


    private boolean isPusherUp = false;
    public static double kpx = 0.058;
    public static double kpy = 0.058;
    public static double kdx = 0.0023;
    public static double kdy = 0.0023;
    public static double kpTheta = 1.3;
    public static double kdTheta = 0.0095;
    public static double kix = 0;
    public static double kiy = 0;
    public static double kitheta = 40000;
    int sorterpos = 0;
    private static final double SORTER_FIRST_POS = 0.0;
    private static final double SORTER_SECOND_POS = 0.45;
    private static final double SORTER_THIRD_POS = 0.90;

    double stage = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        logitechSubsystem = new LogitechSubsystem(hw, "blue");
//        intakeCommand = new IntakeCommand(hw);
        turretSubsystem = new TurretSubsystem(hw);
        outtakeCommand = new OuttakeCommand(hw);

        hw.sorter.setPosition(SORTER_FIRST_POS);
        hw.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.pusher1.setDirection(Servo.Direction.REVERSE);
        hw.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        mecanumCommand.setConstants(kpx, kdx, kix,
                kpy, kdy, kiy,
                kpTheta, kdTheta, kitheta);

        ElapsedTime timer = new ElapsedTime();
        double kp = 0.8;
        double kf = 0.002;
        boolean paused = false;
        boolean submersibleTargetSet = false;
        boolean motorflag = true;
        double target = 2500;
        double current = hw.shooter.getVelocity();
        double error = target - current;

        double power = kp * error + kf * target;


        logitechSubsystem.pattern();

        hw.pusher.setPosition(0);
        hw.pusher1.setPosition(0);
        hw.sorter.setPosition(0);
        hw.light.setPosition(0);
        hw.sorter.setPosition(SORTER_FIRST_POS);


        waitForStart();
        pusherTimer.reset();
        while (opModeIsActive()) {
            telemetry.addLine("for sydney wong");

            mecanumCommand.motorProcess();
            mecanumCommand.processPIDUsingPinpoint();


            mecanumCommand.processOdometry();
            switch (autoState) {
                case MOVEPRELOAD:


                    mecanumCommand.moveToPos(50, 0, 1.85);
                    if (mecanumCommand.isPositionReached()) {
                        mecanumCommand.stop();
                        autoState = AUTO_STATE.PRELOAD_ONE;

                    }
                    break;
                case PRELOAD_ONE:
                    hw.intake.setPower(1.0);
                    hw.shooter.setVelocityPIDFCoefficients(70, 0, 0, 0);
                    outtakeCommand.spinup();
                    if (stage == 0) {
                        pusherTimer.reset();
                        stage++;
                    } else if (stage == 1 && pusherTimer.milliseconds() >= 700) {
                        hw.pusher.setPosition(PUSHER_UP);
                        hw.pusher1.setPosition(PUSHER_UP1);
                        pusherTimer.reset();
                        stage++;
                    } else if (stage == 2 && pusherTimer.milliseconds() >= 300) {
                        hw.pusher.setPosition(PUSHER_DOWN);
                        hw.pusher1.setPosition(PUSHER_DOWN1);
                        pusherTimer.reset();
                        stage++;
                    } else if (stage == 3 && pusherTimer.milliseconds() >= 600) {
                        hw.sorter.setPosition(SORTER_SECOND_POS);
                        autoState = AUTO_STATE.PRELOAD_TWO;
                        pusherTimer.reset();
                        stage = 0;
                    }


                    break;

                case PRELOAD_TWO:
                    hw.intake.setPower(1.0);

                    hw.shooter.setVelocityPIDFCoefficients(67, 0, 0, 0);
                    outtakeCommand.spinup();

                    if (stage == 0 && pusherTimer.milliseconds() >= 500) {
                        hw.pusher.setPosition(PUSHER_UP);
                        hw.pusher1.setPosition(PUSHER_UP1);
                        stage++;
                        pusherTimer.reset();
                    } else if (stage == 1 && pusherTimer.milliseconds() >= 300) {
                        hw.pusher.setPosition(PUSHER_DOWN);
                        hw.pusher1.setPosition(PUSHER_DOWN1);

                        stage++;
                        pusherTimer.reset();
                    } else if (stage == 2 && pusherTimer.milliseconds() >= 600) {
                        pusherTimer.reset();
                        hw.sorter.setPosition(SORTER_THIRD_POS);
                        autoState = AUTO_STATE.PRELOAD_THREE;
                        stage = 0;
                    }


                    break;
                case PRELOAD_THREE:
                    hw.intake.setPower(1.0);


                    hw.shooter.setVelocityPIDFCoefficients(67, 0, 0, 0);
                    outtakeCommand.spinup();

                    if (stage == 0 && pusherTimer.milliseconds() >= 500) {
                        hw.pusher.setPosition(PUSHER_UP);
                        hw.pusher1.setPosition(PUSHER_UP1);

                        stage++;
                        pusherTimer.reset();
                    } else if (stage == 1 && pusherTimer.milliseconds() >= 300) {
                        hw.pusher.setPosition(PUSHER_DOWN);
                        hw.pusher1.setPosition(PUSHER_DOWN1);

                        stage++;
                        pusherTimer.reset();
                    } else if (stage == 2 && pusherTimer.milliseconds() >= 600) {
                        hw.sorter.setPosition(SORTER_FIRST_POS);
                        autoState = AUTO_STATE.PRELOAD_EMPTY;
                        mecanumCommand.moveToPos(110, 0, 0);

                    }

                    break;
                case PRELOAD_EMPTY:
                    hw.intake.setPower(1.0);
                    hw.shooter.setVelocityPIDFCoefficients(67, 0, 0, 0);
                    outtakeCommand.spinup();
                    if (stage == 0) {
                        pusherTimer.reset();
                        stage++;
                    } else if (stage == 1 && pusherTimer.milliseconds() >= 600) {
                        hw.pusher.setPosition(PUSHER_UP);
                        hw.pusher1.setPosition(PUSHER_UP1);
                        pusherTimer.reset();
                        stage++;
                    } else if (stage == 2 && pusherTimer.milliseconds() >= 500) {
                        hw.pusher.setPosition(PUSHER_DOWN);
                        hw.pusher1.setPosition(PUSHER_DOWN1);
                        pusherTimer.reset();
                        stage++;
                    } else if (stage == 3 && pusherTimer.milliseconds() >= 500) {
                        hw.sorter.setPosition(SORTER_SECOND_POS);
                        autoState = AUTO_STATE.INTAKE_MOVE;
                        pusherTimer.reset();
                        stage = 0;
                    }



                    break;
                case INTAKE_MOVE:
                    if (mecanumCommand.isPositionReached()) {
                        hw.intake.setPower(1.0);
                        mecanumCommand.stop();
                        mecanumCommand.moveToPos(120, -50, 1.64);
                        pusherTimer.reset();
                        if (mecanumCommand.isPositionReached() && pusherTimer.milliseconds() >= 500) {

                            hw.sorter.setPosition(SORTER_FIRST_POS);
                            pusherTimer.reset();

                        }
                        autoState = AUTO_STATE.INTAKE_ONE;
                    }
                    break;

                case INTAKE_ONE:
                    hw.intake.setPower(1.0);
                    hw.shooter.setPower(0);
                    hw.sorter.setPosition(SORTER_FIRST_POS);
                    mecanumCommand.moveToPos(120, -64, 1.64);

                    if (mecanumCommand.isPositionReached() && pusherTimer.milliseconds() >= 500) {
                        mecanumCommand.stop();
                        hw.sorter.setPosition(SORTER_SECOND_POS);
                        telemetry.addLine("the big sw");
                        autoState = AUTO_STATE.INTAKE_TWO;
                        pusherTimer.reset();


                    }


                    break;
                case INTAKE_TWO:
                    hw.intake.setPower(1.0);
                    hw.sorter.setPosition(SORTER_SECOND_POS);
                    mecanumCommand.moveToPos(120, -69, 1.64);
                    if (mecanumCommand.isPositionReached() && pusherTimer.milliseconds() >= 700) {
                        mecanumCommand.stop();
                        hw.sorter.setPosition(SORTER_THIRD_POS);
                        autoState = AUTO_STATE.INTAKE_THREE;
                        pusherTimer.reset();


                    }


                    break;


                case INTAKE_THREE:
                    hw.intake.setPower(1.0);
                    hw.sorter.setPosition(SORTER_THIRD_POS);
                    mecanumCommand.moveToPos(120, -75, 1.64);

                    if (mecanumCommand.isPositionReached() && pusherTimer.milliseconds() >= 600) {
                        mecanumCommand.stop();
                        hw.sorter.setPosition(SORTER_THIRD_POS);
                        telemetry.addLine("the big sw");
                        autoState = AUTO_STATE.SUBMERSIBLE_PICKUP;
                        pusherTimer.reset();


                    }
                    break;

                case SUBMERSIBLE_PICKUP:
                    mecanumCommand.moveToPos(50, 0, 1.85);
                    hw.sorter.setPosition(SORTER_FIRST_POS);
                    if (mecanumCommand.isPositionReached()) {
                        mecanumCommand.stop();
                        stage = 0;
                        autoState = AUTO_STATE.INTAKE_SHOOT;

                    }
                    break;
                case INTAKE_SHOOT:
                    hw.intake.setPower(1.0);
                    hw.shooter.setVelocityPIDFCoefficients(70 , 0, 0, 0);
                    outtakeCommand.spinup();
                    if (stage == 0) {
                        pusherTimer.reset();
                        stage++;
                    } else if (stage == 1 && pusherTimer.milliseconds() >= 600) {
                        hw.pusher.setPosition(PUSHER_UP);
                        hw.pusher1.setPosition(PUSHER_UP1);
                        pusherTimer.reset();
                        stage++;
                    } else if (stage == 2 && pusherTimer.milliseconds() >= 500) {
                        hw.pusher.setPosition(PUSHER_DOWN);
                        hw.pusher1.setPosition(PUSHER_DOWN1);
                        pusherTimer.reset();
                        stage++;
                    } else if (stage == 3 && pusherTimer.milliseconds() >= 500) {
                        hw.sorter.setPosition(SORTER_SECOND_POS);
                        autoState = AUTO_STATE.INTAKE_SHOOT1;
                        pusherTimer.reset();
                        stage = 0;
                    }



                    break;
                case INTAKE_SHOOT1:
                    hw.intake.setPower(1.0);
                    hw.shooter.setVelocityPIDFCoefficients(67, 0, 0, 0);
                    outtakeCommand.spinup();
                    if (stage == 0) {
                        pusherTimer.reset();
                        stage++;
                    } else if (stage == 1 && pusherTimer.milliseconds() >= 500) {
                        hw.pusher.setPosition(PUSHER_UP);
                        hw.pusher1.setPosition(PUSHER_UP1);
                        pusherTimer.reset();
                        stage++;
                    } else if (stage == 2 && pusherTimer.milliseconds() >= 500) {
                        hw.pusher.setPosition(PUSHER_DOWN);
                        hw.pusher1.setPosition(PUSHER_DOWN1);
                        pusherTimer.reset();
                        stage++;
                    } else if (stage == 3 && pusherTimer.milliseconds() >= 500) {
                        hw.sorter.setPosition(SORTER_THIRD_POS);
                        autoState = AUTO_STATE.INTAKE_SHOOT2;
                        pusherTimer.reset();
                        stage = 0;
                    }


                    break;
                case INTAKE_SHOOT2:
                    hw.intake.setPower(1.0);


                    hw.shooter.setVelocityPIDFCoefficients(67, 0, 0, 0);
                    outtakeCommand.spinup();

                    if (stage == 0 && pusherTimer.milliseconds() >= 500) {
                        hw.pusher.setPosition(PUSHER_UP);
                        hw.pusher1.setPosition(PUSHER_UP1);

                        stage++;
                        pusherTimer.reset();
                    } else if (stage == 1 && pusherTimer.milliseconds() >= 300) {
                        hw.pusher.setPosition(PUSHER_DOWN);
                        hw.pusher1.setPosition(PUSHER_DOWN1);

                        stage++;
                        pusherTimer.reset();
                    } else if (stage == 2 && pusherTimer.milliseconds() >= 600) {
                        hw.sorter.setPosition(SORTER_FIRST_POS);
                        autoState = AUTO_STATE.CLEAR;

                    }
                    break;
                case CLEAR:
                    hw.intake.setPower(1.0);
                    hw.shooter.setVelocityPIDFCoefficients(67, 0, 0, 0);
                    outtakeCommand.spinup();
                    if (stage == 0) {
                        pusherTimer.reset();
                        stage++;
                    } else if (stage == 1 && pusherTimer.milliseconds() >= 600) {
                        hw.pusher.setPosition(PUSHER_UP);
                        hw.pusher1.setPosition(PUSHER_UP1);
                        pusherTimer.reset();
                        stage++;
                    } else if (stage == 2 && pusherTimer.milliseconds() >= 500) {
                        hw.pusher.setPosition(PUSHER_DOWN);
                        hw.pusher1.setPosition(PUSHER_DOWN1);
                        pusherTimer.reset();
                        stage++;
                    } else if (stage == 3 && pusherTimer.milliseconds() >= 500) {
                        hw.sorter.setPosition(SORTER_SECOND_POS);
                        autoState = AUTO_STATE.NINEBALL_1;
                        pusherTimer.reset();
                        stage = 0;
                    }



                    break;
                case NINEBALL_1:
                    if (mecanumCommand.isPositionReached()) {
                        hw.intake.setPower(1.0);
                        mecanumCommand.stop();
                        mecanumCommand.moveToPos(180, -50, 1.64);
                        pusherTimer.reset();
                        if (mecanumCommand.isPositionReached() && pusherTimer.milliseconds() >= 500) {

                            hw.sorter.setPosition(SORTER_FIRST_POS);
                            pusherTimer.reset();

                        }
                    }
                    break;



                case LEAVE:
                    mecanumCommand.moveToPos(110,0,0);
                        if(mecanumCommand.isPositionReached()){
                            mecanumCommand.stop();
                        }

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
        telemetry.addData("state: ", autoState);

        telemetry.update();
    }
}






