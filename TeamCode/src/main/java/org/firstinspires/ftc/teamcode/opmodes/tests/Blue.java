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
    private Hardware hw;

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
        RETURN,
        CLEAR,
        NINEBALL_1,
        NINEBALL_2,
        NINEBALL_3,
        TURN_ONE,
        SUBMERSIBLE_PICKUP,
        LEAVE,
        NINEBALL_SHOOT1,
        NINEBALL_SHOOT2,
        NINEBALL_SHOOT3,
        NINEBALL_EMPTY,
        PICKUP_FIRST,

    }

    AUTO_STATE autoState = AUTO_STATE.MOVEPRELOAD;
    private static final double PUSHER_UP = 0.39;
    private static final double PUSHER_DOWN = 0.0;
    private static final double PUSHER_UP1 = 0.19;
    private static final double PUSHER_DOWN1 = 0;
    private static final long PUSHER_TIME = 300;
    private final ElapsedTime pusherTimer = new ElapsedTime();
    private boolean delayTimerStarted = false;


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

    private int stage1 = 0;

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

        hw.pusher.setPosition(0.0);
        hw.pusher1.setPosition(0.0);
        hw.sorter.setPosition(0);
//        hw.light.setPosition(0);


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
                    processMovePreload();
                    break;
                case PRELOAD_ONE:
                    processPreloadOne();
                    break;
                case PRELOAD_TWO:
                    processPreloadTwo();
                    break;
                case PRELOAD_THREE:
                    processPreloadThree();
                    break;
                case PRELOAD_EMPTY:
                    processPreloadEmpty();
                case INTAKE_MOVE:
                    processIntakeMove();
                    break;
                case INTAKE_ONE:
                    processIntakeOne();
                    break;
                case INTAKE_TWO:
                    processIntakeTwo();
                    break;
                case INTAKE_THREE:
                    processIntakeThree();
                    break;
                case SUBMERSIBLE_PICKUP:
                    processSubPickup();
                    break;
                case INTAKE_SHOOT:
                    processShoot();
                    break;
                case INTAKE_SHOOT1:
                    processShoot1();
                    break;
                case INTAKE_SHOOT2:
                    processShoot2();
                    break;
                case LEAVE:
                    processLeave();
                    break;


                default:
                    updateTelemetry();
                    break;
            }
        }
    }


    private void processMovePreload() {
        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(61, 44, 3 * (Math.PI / 4));
                if (mecanumCommand.isPositionReached()) {
                    mecanumCommand.stop();
                }

                break;
            case 1:
                stage1 = 0;

                autoState = AUTO_STATE.PRELOAD_ONE;
        }
    }

    private void processPreloadOne() {
        hw.intake.setPower(1.0);
        hw.shooter.setVelocityPIDFCoefficients(60, 0, 0, 0);
        outtakeCommand.spinup();
        switch (stage1) {
            case 0:
                pusherTimer.reset();
                stage1++;
                break;
            case 1:
                hw.pusher.setPosition(PUSHER_UP);
                hw.pusher1.setPosition(PUSHER_UP1);
                //wait time
                stage1++;
                break;
            case 2:
                hw.pusher.setPosition(PUSHER_DOWN);
                hw.pusher1.setPosition(PUSHER_DOWN1);
                //wait time
                stage1++;
                break;
            case 3:
                hw.sorter.setPosition(SORTER_SECOND_POS);
                autoState = AUTO_STATE.PRELOAD_TWO;
                //wait time
                stage1 = 0;
                break;


        }


    }

    private void processPreloadTwo() {
        hw.intake.setPower(1.0);
        hw.shooter.setVelocityPIDFCoefficients(67, 0, 0, 0);
        outtakeCommand.spinup();
        switch (stage1) {
            case 0:
                hw.pusher.setPosition(PUSHER_UP);
                hw.pusher1.setPosition(PUSHER_UP1);
                stage1++;
                //wait time
                break;
            case 1:
                hw.pusher.setPosition(PUSHER_DOWN);
                hw.pusher1.setPosition(PUSHER_DOWN1);
                //wait time
                stage1++;
                break;
            case 2:
                hw.sorter.setPosition(SORTER_THIRD_POS);
                autoState = AUTO_STATE.PRELOAD_THREE;
                // wait time
                stage1 = 0;
                break;

        }
    }

    private void processPreloadThree() {
        hw.intake.setPower(1.0);
        hw.shooter.setVelocityPIDFCoefficients(67, 0, 0, 0);
        outtakeCommand.spinup();
        switch (stage1) {
            case 0:
                hw.pusher.setPosition(PUSHER_UP);
                hw.pusher1.setPosition(PUSHER_UP1);
                stage1++;
                //wait time
                break;
            case 1:
                hw.pusher.setPosition(PUSHER_DOWN);
                hw.pusher1.setPosition(PUSHER_DOWN1);
                //wait time
                stage1++;
                break;
            case 2:
                hw.sorter.setPosition(SORTER_FIRST_POS);
                autoState = AUTO_STATE.PRELOAD_EMPTY;
                // wait time
                stage1 = 0;
                break;

        }

    }

    private void processPreloadEmpty() {
        hw.intake.setPower(1.0);
        hw.shooter.setVelocityPIDFCoefficients(67, 0, 0, 0);
        outtakeCommand.spinup();
        switch (stage1) {
            case 0:
                hw.pusher.setPosition(PUSHER_UP);
                hw.pusher1.setPosition(PUSHER_UP1);
                stage1++;
                //wait time
                break;
            case 1:
                hw.pusher.setPosition(PUSHER_DOWN);
                hw.pusher1.setPosition(PUSHER_DOWN1);
                //wait time
                stage1++;
                break;
            case 2:
                hw.sorter.setPosition(SORTER_FIRST_POS);
                autoState = AUTO_STATE.INTAKE_MOVE;
                // wait time
                stage1 = 0;
                break;

        }

    }

    private void processIntakeMove() {
        hw.intake.setPower(1.0);
        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(112, 39, Math.PI / 2);
                stage1++;
                break;
            case 1:
                hw.sorter.setPosition(SORTER_SECOND_POS);
                autoState = AUTO_STATE.INTAKE_ONE;
                stage1 = 0;
                break;


        }

    }

    private void processIntakeOne() {
        hw.intake.setPower(1.0);
        hw.shooter.setPower(0);
        hw.sorter.setPosition(SORTER_FIRST_POS);

        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(112, 32, Math.PI / 2);
                if (mecanumCommand.isPositionReached()) {
                    //wait
                    stage1++;

                }
                break;
            case 1:
                hw.sorter.setPosition(SORTER_SECOND_POS);
                //wait time
                stage1++;
                autoState = AUTO_STATE.INTAKE_TWO;
                stage1 = 0;
                break;

        }


    }

    private void processIntakeTwo() {
        hw.intake.setPower(1.0);
        hw.shooter.setPower(0);
        hw.sorter.setPosition(SORTER_FIRST_POS);

        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(112, 32, Math.PI / 2);
                if (mecanumCommand.isPositionReached()) {
                    //wait 
                    stage1++;

                }
                break;
            case 1:
                hw.sorter.setPosition(SORTER_THIRD_POS);
                //wait time
                stage1++;
                autoState = AUTO_STATE.INTAKE_THREE;
                stage1 = 0;
                break;

        }


    }

    private void processIntakeThree() {
        hw.intake.setPower(1.0);
        hw.shooter.setPower(0);
        hw.sorter.setPosition(SORTER_FIRST_POS);
        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(112, 32, Math.PI / 2);
                if (mecanumCommand.isPositionReached()) {
                    //wait
                    stage1++;

                }
                break;
            case 1:
                hw.sorter.setPosition(SORTER_THIRD_POS);
                //wait time
                stage1++;
                autoState = AUTO_STATE.SUBMERSIBLE_PICKUP;
                stage1 = 0;
                break;

        }


    }

    private void processSubPickup() {
        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(61, 44, 3 * (Math.PI / 4));
                if (mecanumCommand.isPositionReached()) {
                    mecanumCommand.stop();
                }

                break;
            case 1:
                stage1 = 0;

                autoState = AUTO_STATE.INTAKE_SHOOT1;
        }
    }

    private void processShoot() {
        hw.intake.setPower(1.0);
        hw.shooter.setVelocityPIDFCoefficients(70, 0, 0, 0);
        outtakeCommand.spinup();
        switch (stage1) {
            case 0:
                //wait time
                hw.pusher.setPosition(PUSHER_UP);
                hw.pusher1.setPosition(PUSHER_UP1);
                stage1++;
                break;
            case 1:
                //wait time
                hw.pusher.setPosition(PUSHER_DOWN);
                hw.pusher1.setPosition(PUSHER_DOWN1);
                stage1++;
                break;
            case 2:
                //wait time
                hw.sorter.setPosition(SORTER_SECOND_POS);
                stage1 = 0;
                autoState = AUTO_STATE.INTAKE_SHOOT1;
                break;


        }

    }

    private void processShoot1() {
        hw.intake.setPower(1.0);
        hw.shooter.setVelocityPIDFCoefficients(70, 0, 0, 0);
        outtakeCommand.spinup();
        switch (stage1) {
            case 0:
                //wait time
                hw.pusher.setPosition(PUSHER_UP);
                hw.pusher1.setPosition(PUSHER_UP1);
                stage1++;
                break;
            case 1:
                //wait time
                hw.pusher.setPosition(PUSHER_DOWN);
                hw.pusher1.setPosition(PUSHER_DOWN1);
                stage1++;
                break;
            case 2:
                //wait time
                hw.sorter.setPosition(SORTER_THIRD_POS);
                stage1 = 0;
                autoState = AUTO_STATE.INTAKE_SHOOT2;
                break;


        }

    }

    private void processShoot2() {
        hw.intake.setPower(1.0);
        hw.shooter.setVelocityPIDFCoefficients(70, 0, 0, 0);
        outtakeCommand.spinup();
        switch (stage1) {
            case 0:
                //wait time
                hw.pusher.setPosition(PUSHER_UP);
                hw.pusher1.setPosition(PUSHER_UP1);
                stage1++;
                break;
            case 1:
                //wait time
                hw.pusher.setPosition(PUSHER_DOWN);
                hw.pusher1.setPosition(PUSHER_DOWN1);
                stage1++;
                break;
            case 2:
                //wait time
                hw.sorter.setPosition(SORTER_FIRST_POS);
                stage1 = 0;
                autoState = AUTO_STATE.LEAVE;
                break;


        }
    }
    private void processLeave(){

        hw.shooter.setVelocityPIDFCoefficients(70, 0, 0, 0);
        outtakeCommand.stopShooter();
        switch (stage1){
            case 0:
                //wait time
                mecanumCommand.moveToPos(112, 32, Math.PI / 2);
                if(mecanumCommand.isPositionReached()) {
                    stage1++;
                }
                break;
            case 1:
                mecanumCommand.stop();
                break;



        }

    }


    public void updateTelemetry() {
        telemetry.addData("x: ", mecanumCommand.getOdoX());
        telemetry.addData("y: ", mecanumCommand.getOdoY());
        telemetry.addData("Theta: ", mecanumCommand.getOdoHeading());
        telemetry.addData("state: ", autoState);

        telemetry.update();
    }
    private boolean waitDelay(double waitTimeMs) {
        if (!delayTimerStarted) {
            pusherTimer.reset();
            delayTimerStarted = true;
            return false;
        } else {
            if (pusherTimer.milliseconds() >= waitTimeMs) {
                delayTimerStarted = false;
                return true;
            }
            return false;
        }
    }

}






