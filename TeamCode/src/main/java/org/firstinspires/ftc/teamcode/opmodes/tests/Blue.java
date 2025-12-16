/**
*thank you kirby for everything
 */

package org.firstinspires.ftc.teamcode.opmodes.tests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.cameras.IntakeSubsystem;


import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.odometry.PinPointOdometrySubsystem;


@Autonomous(name = "Blue Auto")
public class Blue extends LinearOpMode {
    private Hardware hw;

    ElapsedTime timer;
    private MecanumCommand mecanumCommand;
    private TurretSubsystem turretSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private OuttakeCommand outtakeCommand;
    private PinPointOdometrySubsystem odo;
    boolean firstInstance = true;

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
        SIX_BALL,
        INTAKE_MOVE2,
        INTAKE_ONE2,
        INTAKE_TWO2,
        INTAKE_THREE2,
        NINE_BALL,
        LEAVE,


    }


    AUTO_STATE autoState = AUTO_STATE.MOVEPRELOAD;


    private static final double PUSHER_UP = 0.18;
    private static final double PUSHER_DOWN = 0.0;
    private static final double PUSHER_UP1 = 0.18;
    private static final double PUSHER_DOWN1 = 0;
    private static final long PUSHER_TIME = 300;
    private final ElapsedTime pusherTimer = new ElapsedTime();
    private boolean delayTimerStarted = false;


    private boolean isPusherUp = false;
    public static double kpx = 0.055;
    public static double kpy = 0.055;
    public static double kdx = 0.0029;
    public static double kdy = 0.0029;
    public static double kpTheta = 1.45;
    public static double kdTheta = 0.0095;
    public static double kix = 0;
    public static double kiy = 0;
    public static double kitheta = 40000;
    int sorterpos = 0;
    private static final double SORTER_FIRST_POS = 0.0;
    private static final double SORTER_SECOND_POS = 0.45;
    private static final double SORTER_THIRD_POS = 0.90;
    Servo pusher;
    Servo pusher1;
    Servo sorter;
    Servo stopper;


    private int stage1 = 0;

    double stage = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);

        intakeSubsystem = new IntakeSubsystem(hw);

        turretSubsystem = new TurretSubsystem(hw);
        outtakeCommand = new OuttakeCommand(hw);
        odo = new PinPointOdometrySubsystem(hw);


        timer = new ElapsedTime();
        hw.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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


//        hw.light.setPosition(0);


        pusher = hardwareMap.get(Servo.class, "pusher");
        pusher1 = hardwareMap.get(Servo.class, "pusher1");
        sorter = hardwareMap.get(Servo.class, "sorter");
        stopper = hardwareMap.get(Servo.class, "stopper");

        pusher.setPosition(0.0);
        pusher1.setPosition(0.0);
        sorter.setPosition(0);
        waitForStart();
        pusherTimer.reset();
        pusher1.setDirection(Servo.Direction.REVERSE);
        sorter.setPosition(SORTER_FIRST_POS);
//        odo.reset();

        while (opModeIsActive()) {
            updateTelemetry();

            mecanumCommand.motorProcess();
            mecanumCommand.processPIDUsingPinpoint();
            mecanumCommand.deadReckoning();

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
                case SIX_BALL:
                    processSixBall();
                    break;
                case INTAKE_MOVE2:
                    intakeMove2();
                    break;
//                case INTAKE_ONE2:
//                    intakeOne2();
//                    break;
//                case INTAKE_TWO2:
//                    intakeTwo2();
//                    break;
//                case INTAKE_THREE2:
//                    intakeThree2();
//                    break;
//                case NINE_BALL:
//                    nineBall();
//                    break;
//                case LEAVE:
//                    leave();
//                    break;


                default:
                    updateTelemetry();
                    break;
            }
        }
    }


    private void processMovePreload() {
        intakeSubsystem.intake();
        outtakeCommand.spinup();
        switch (stage1) {
            case 0:
                stopper.setPosition(0.5);
                mecanumCommand.moveToPos(85, 58, 2.27);
                if (mecanumCommand.isPositionReached()) {
                    stage1++;
                }

                break;
            case 1:
                stopper.setPosition(0.5);
                pusher.setPosition(PUSHER_UP);
                pusher1.setPosition(PUSHER_UP1);
                waitTime(300);
                break;
            case 2:
                pusher.setPosition(PUSHER_DOWN);
                pusher1.setPosition(PUSHER_DOWN1);
                waitTime(300);
            case 3:
                stage1 = 0;
                autoState = AUTO_STATE.PRELOAD_ONE;
        }
    }

    private void processPreloadOne() {
        switch (stage1) {
            case 0:
                waitTime(300);
                stopper.setPosition(0.5);
                break;
            case 1:
                pusher.setPosition(PUSHER_UP);
                pusher1.setPosition(PUSHER_UP1);
                waitTime(300);
                break;
            case 2:
                pusher.setPosition(PUSHER_DOWN);
                pusher1.setPosition(PUSHER_DOWN1);
                waitTime(300);
                break;
            case 3:
                sorter.setPosition(SORTER_SECOND_POS);
                waitTime(500);
                break;

            case 4:
                autoState = AUTO_STATE.PRELOAD_TWO;
                //wait time
                stage1 = 0;
                break;


        }


    }

    private void processPreloadTwo() {

//        hw.shooter.setVelocityPIDFCoefficients(67, 0, 0, 0);
        outtakeCommand.spinup();
        outtakeCommand.spinup();
        stopper.setPosition(0.5);
        switch (stage1) {
//            case 0:
//                pusher.setPosition(PUSHER_UP);
//                pusher1.setPosition(PUSHER_UP1);
//                waitTime(200);
//                break;
//            case 1:
//
//                pusher.setPosition(PUSHER_DOWN);
//                pusher1.setPosition(PUSHER_DOWN1);
//                waitTime(500);
//                //wait time
//
//                break;
            case 0:
                pusher.setPosition(PUSHER_UP);
                pusher1.setPosition(PUSHER_UP1);
                waitTime(300);
                break;
            case 1:

                pusher.setPosition(PUSHER_DOWN);
                pusher1.setPosition(PUSHER_DOWN1);
                waitTime(300);
                //wait time

                break;
            case 2:
                sorter.setPosition(SORTER_THIRD_POS);
                waitTime(500);
                break;

            case 3:
                autoState = AUTO_STATE.PRELOAD_THREE;
                // wait time
                stage1 = 0;
                break;


        }
    }

    private void processPreloadThree() {

//        hw.shooter.setVelocityPIDFCoefficients(67, 0, 0, 0);
        outtakeCommand.spinup();
        stopper.setPosition(0.5);
        switch (stage1) {
            case 0:
                pusher.setPosition(PUSHER_UP);
                pusher1.setPosition(PUSHER_UP1);
                waitTime(500);

                //wait time
                break;
            case 1:
                pusher.setPosition(PUSHER_DOWN);
                pusher1.setPosition(PUSHER_DOWN1);
                waitTime(500);
                //wait time

                break;
            case 2:
                sorter.setPosition(SORTER_FIRST_POS);
                waitTime(500);
                break;
            case 3:
                autoState = AUTO_STATE.INTAKE_MOVE;
                // wait time
                stage1 = 0;
                break;

        }

    }

    private void processIntakeMove() {
        intakeSubsystem.intake();
        outtakeCommand.stopShooter();
        stopper.setPosition(0);
        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(118, 40, Math.PI / 2);
                if (mecanumCommand.isPositionReached()) {
                    stage1++;
                }
                break;
            case 1:
                autoState = AUTO_STATE.INTAKE_ONE;
                stage1 = 0;
                break;


        }

    }

    private void processIntakeOne() {
        intakeSubsystem.intake();
        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(112, 15, Math.PI / 2);
                if (mecanumCommand.isPositionReached()) {
                    mecanumCommand.moveToPos(112, 15, Math.PI / 2);
                    stage1++;

                }
                break;
            case 1:
                waitTime(500);
                break;
            case 2:
                sorter.setPosition(SORTER_SECOND_POS);
                waitTime(800);
                break;
            case 3:
                autoState = AUTO_STATE.INTAKE_TWO;
                stage1 = 0;
                break;

        }


    }

    private void processIntakeTwo() {
        intakeSubsystem.intake();

        switch (stage1) {
            case 0:
                waitTime(400);
                mecanumCommand.moveToPos(112, 4, Math.PI / 2);
                if (mecanumCommand.isPositionReached()) {
                    mecanumCommand.moveToPos(112, 5.5, Math.PI / 2);
                    stage1++;
                }

                break;
            case 1:
                waitTime(500);
                break;
            case 2:
                sorter.setPosition(SORTER_THIRD_POS);
                waitTime(800);
                break;
            case 3:
                autoState = AUTO_STATE.INTAKE_THREE;
                stage1 = 0;
                break;

        }


    }

    private void processIntakeThree() {
        intakeSubsystem.intake();
        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(112, -8, Math.PI / 2);
                if (mecanumCommand.isPositionReached()) {
                    mecanumCommand.moveToPos(112, -20, Math.PI / 2);
                    stage1++;
                }
                break;
            case 1:
                waitTime(500);
                break;
            case 2:
                stage1 = 0;
                autoState = AUTO_STATE.SIX_BALL;
                break;
        }


    }

    private void processSixBall() {
        outtakeCommand.spinup();
        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(87, 58, 2.27);
                if (mecanumCommand.isPositionReached()) {
                    mecanumCommand.stop();
                    sorter.setPosition(SORTER_FIRST_POS);
                    stage1++;
                }

                break;
            case 1:
                waitTime(300);
                stopper.setPosition(0.5);
                break;
            case 2:
                pusher.setPosition(PUSHER_UP);
                pusher1.setPosition(PUSHER_UP1);
                waitTime(500);
                break;
            case 3:
                pusher.setPosition(PUSHER_DOWN);
                pusher1.setPosition(PUSHER_DOWN1);
                waitTime(300);
                break;
            case 4:
                sorter.setPosition(SORTER_SECOND_POS);
                waitTime(500);
                break;
            case 5:
                pusher.setPosition(PUSHER_UP);
                pusher1.setPosition(PUSHER_UP1);
                waitTime(500);
                break;
            case 6:
                pusher.setPosition(PUSHER_DOWN);
                pusher1.setPosition(PUSHER_DOWN1);
                waitTime(300);
                break;
            case 7:
                sorter.setPosition(SORTER_THIRD_POS);
                waitTime(500);
                break;
            case 8:
                pusher.setPosition(PUSHER_UP);
                pusher1.setPosition(PUSHER_UP1);
                waitTime(500);
                break;
            case 9:
                pusher.setPosition(PUSHER_DOWN);
                pusher1.setPosition(PUSHER_DOWN1);
                waitTime(300);
                break;
            case 10:
                pusher.setPosition(PUSHER_UP);
                pusher1.setPosition(PUSHER_UP1);
                waitTime(500);
                break;
            case 11:
                pusher.setPosition(PUSHER_DOWN);
                pusher1.setPosition(PUSHER_DOWN1);
                waitTime(300);
                break;
            case 12:
                autoState = AUTO_STATE.INTAKE_MOVE2;
                outtakeCommand.stopShooter();
                stage1 = 0;
                break;

        }
    }

    private void intakeMove2() {
        intakeSubsystem.intake();
        stopper.setPosition(0);
        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(112, -8, Math.PI / 2);
                if (mecanumCommand.isPositionReached()) {
                    mecanumCommand.stop();
                    stage1++;
                }
                break;
            case 1:
//                autoState = AUTO_STATE.INTAKE_ONE2;
                stage1 = 0;
                break;


        }

    }

    private void intakeOne2() {
        intakeSubsystem.intake();
        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(180, 16, Math.PI / 2);
                if (mecanumCommand.isPositionReached()) {
                    mecanumCommand.moveToPos(180, 18, Math.PI / 2);

                    stage1++;

                }
                break;
            case 1:
                waitTime(300);
                break;
            case 2:
                sorter.setPosition(SORTER_SECOND_POS);
                waitTime(500);
                break;
            case 3:
                autoState = AUTO_STATE.INTAKE_TWO2;
                stage1 = 0;
                break;

        }


    }

    private void intakeTwo2() {
        intakeSubsystem.intake();

        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(180, 10, Math.PI / 2);
                if (mecanumCommand.isPositionReached()) {
                    mecanumCommand.moveToPos(180, 12, Math.PI / 2);
                    stage1++;
                }

                break;
            case 1:
                waitTime(300);//changed delay
                break;
            case 2:
                sorter.setPosition(SORTER_THIRD_POS);
                waitTime(600);
                break;
            case 3:
                autoState = AUTO_STATE.INTAKE_THREE2;
                stage1 = 0;
                break;

        }


    }

    private void intakeThree2() {
        intakeSubsystem.intake();
        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(180, -9, Math.PI / 2);
                if (mecanumCommand.isPositionReached()) {
                    mecanumCommand.moveToPos(180, -7, Math.PI / 2);
                    stage1++;
                }
                break;
            case 1:
                waitTime(400);
                break;
            case 2:
                stage1 = 0;
                autoState = AUTO_STATE.NINE_BALL;
                break;


        }


    }

    private void nineBall() {
        outtakeCommand.spinup();
        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(80, 48, 3 * (Math.PI / 4));
                if (mecanumCommand.isPositionReached()) {
                    sorter.setPosition(SORTER_FIRST_POS);
                    stage1++;
                }

                break;
            case 1:
                waitTime(300);
                stopper.setPosition(0.5);
                break;
            case 2:
                pusher.setPosition(PUSHER_UP);
                pusher1.setPosition(PUSHER_UP1);
                waitTime(200);
                break;
            case 3:
                pusher.setPosition(PUSHER_DOWN);
                pusher1.setPosition(PUSHER_DOWN1);
                waitTime(200);
                break;
            case 4:
                sorter.setPosition(SORTER_SECOND_POS);
                waitTime(500);
                break;

            case 5:
                pusher.setPosition(PUSHER_UP);
                pusher1.setPosition(PUSHER_UP1);
                waitTime(200);
                break;
            case 6:
                pusher.setPosition(PUSHER_DOWN);
                pusher1.setPosition(PUSHER_DOWN1);
                waitTime(200);
                break;
            case 7:
                sorter.setPosition(SORTER_THIRD_POS);
                waitTime(500);
                break;
            case 8:
                pusher.setPosition(PUSHER_UP);
                pusher1.setPosition(PUSHER_UP1);
                waitTime(200);
                break;
            case 9:
                pusher.setPosition(PUSHER_DOWN);
                pusher1.setPosition(PUSHER_DOWN1);
                waitTime(200);
                break;
            case 10:
                autoState = AUTO_STATE.LEAVE;
                stage1 = 0;
                break;
        }


    }

    private void leave() {
        switch (stage1) {
            case 0:
                mecanumCommand.moveToPos(180, 22, Math.PI / 2);
                if (mecanumCommand.isPositionReached()) {
                    mecanumCommand.stop();

                }

        }
    }


    public void updateTelemetry() {
        telemetry.addData("x: ", mecanumCommand.getOdoX());
        telemetry.addData("y: ", mecanumCommand.getOdoY());
        telemetry.addData("Thetareading: ", mecanumCommand.getOdoHeading());
        telemetry.addData("state: ", autoState);
        telemetry.addData("stage: ", stage1);
        telemetry.addData("xFinal: ", mecanumCommand.xFinal);
        telemetry.addData("yFinal: ", mecanumCommand.yFinal);
        telemetry.addData("thetaFinal: ", mecanumCommand.thetaFinal);

        telemetry.update();
    }


    private void waitTime(double milliseconds) {
        if (firstInstance) {
            timer.reset();
            firstInstance = false;
        }
        if (timer.milliseconds() > milliseconds && !isStopRequested()) {
            firstInstance = true;
            stage1++;
        }
    }

}






