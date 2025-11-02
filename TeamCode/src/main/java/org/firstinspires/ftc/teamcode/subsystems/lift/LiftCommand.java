package org.firstinspires.ftc.teamcode.subsystems.lift;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeSubsystem;

public class LiftCommand {
    private LiftSubsystem liftSubsystem;
    private OuttakeSubsystem outtakeSubsystem;
    private Hardware hw;
    private double power;

    private ElapsedTime elapsedTime;

    public LiftCommand(Hardware hw) {
        this.hw = hw;
        this.liftSubsystem = new LiftSubsystem(hw);
        this.outtakeSubsystem = new OuttakeSubsystem(hw);
        hw.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        elapsedTime = new ElapsedTime();
        this.power = 0;


        elapsedTime = new ElapsedTime();
    }

    public void stopintake() {
        liftSubsystem.stopintake();
    }

    public void handleIntake() {
        liftSubsystem.intake();
    }

//    public void turn(){ liftSubsystem.turretTurn();}
//    public void turn2(){
//        liftSubsystem.turretTurn2();
//    }
//    public void turn3(){
//        liftSubsystem.turretTurn3();
//    }
    public void shoot(){
        double targetRPM = 5600;
        double currentRPM = hw.shooter.getVelocity();
        double output = outtakeSubsystem.outputPositional(targetRPM, currentRPM);
        hw.shooter.setPower(output);

    }
    public void push(){
        liftSubsystem.push();
    }
    public void pull(){
        liftSubsystem.pull();
    }
    public void shootstop(){
        liftSubsystem.shooterstop();
    }
//    public void rainbet(){
//        liftSubsystem.rainbetIntake();
//    }
//    public void rainbetStop(){
//        liftSubsystem.stopTurn();
//    }






}




