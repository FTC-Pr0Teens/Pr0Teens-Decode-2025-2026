package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.sorting.SortingSubsystem;

import java.util.concurrent.CompletableFuture;

public class TeleOpTest extends LinearOpMode {

    private MecanumCommand mecanumCommand;
    private SortingSubsystem sortingSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private Hardware hw;


    private final ElapsedTime timer = new ElapsedTime();

    private int numRotations = 0;
    private int numArtifactsFired = 0;

    enum SHOOTING_STATE{
        IDLE,
        SHOOTING,
        RAMP_UP,
        FIRE,
        RELOAD
    }
    SHOOTING_STATE shootingState = SHOOTING_STATE.IDLE;

    private final int CLOSE_RANGE_VEL = 2300;
    private int currentVel;

    public void runOpMode() throws InterruptedException {
        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        sortingSubsystem = new SortingSubsystem(hw);
        intakeSubsystem = new IntakeSubsystem(hw);


        //btw completablefuture only allows up to 3 threads so if you want to use more, use packets (check old code)
        CompletableFuture.runAsync(this::processShooterState);
        CompletableFuture.runAsync(this::processShooterVelocity);
        while (opModeIsActive()) {
            mecanumCommand.fieldOrientedMove(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

    }

    private void processShooterState(){
        switch (shootingState){
            case IDLE:
                if (gamepad1.right_trigger > 0.5) {
                    intakeSubsystem.intake();
                } else if (gamepad1.left_trigger > 0.5){
                    intakeSubsystem.reverse();
                }

                if (gamepad1.left_bumper){
                    sortingSubsystem.rotate();
                    numRotations++;
                    waitTime(250);
                }

                if (numRotations == 3){
                    shootingState = SHOOTING_STATE.SHOOTING;
                }
                break;
            case SHOOTING:
                if (gamepad1.right_trigger > 0.5){
                    shootingState = SHOOTING_STATE.RAMP_UP;
                    numArtifactsFired++;
                }

                //in case driver rotates spindexer multiple times
                if (gamepad1.right_trigger > 0.5 || gamepad1.left_trigger > 0.5) {
                    shootingState = SHOOTING_STATE.IDLE;
                }

                break;
            case RAMP_UP:
                currentVel = CLOSE_RANGE_VEL;
                //TODO: implement if targetVelocity is reached, move on to next TRANSFER STATE
                break;
            case FIRE:
                //TODO:tune waittimes to maximize efficiency
                sortingSubsystem.pusherUp();
                waitTime(500);
                sortingSubsystem.pusherDown();
                waitTime(500);
                break;
            case RELOAD:
                //TODO:tune waittimes to maximize efficiency
                sortingSubsystem.rotate();
                waitTime(500);
                shootingState = SHOOTING_STATE.SHOOTING;

                if (numArtifactsFired == 3){
                    shootingState = SHOOTING_STATE.IDLE;
                    currentVel = 0;
                }
                break;
        }
    }

    private void waitTime(double milliseconds){
        timer.reset();
        while (timer.milliseconds() < milliseconds && !isStopRequested()){

        }
    }
    private void processShooterVelocity(){
        while (opModeIsActive()){
            //TODO: set shooter velocity to currentRPM (outtakeCommand.setVelocity(currentRPM))
        }
    }


    public void processTelemetry() {
        telemetry.addData("state ", shootingState);
        telemetry.addData("currentVel ", currentVel);
        telemetry.addData("numArtifactsFired ", numArtifactsFired);
        telemetry.addData("numRotations ", numRotations);
        telemetry.update();
    }
}
