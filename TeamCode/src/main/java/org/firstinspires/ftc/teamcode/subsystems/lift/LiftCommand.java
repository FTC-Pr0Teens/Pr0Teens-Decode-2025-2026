package org.firstinspires.ftc.teamcode.subsystems.lift;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;

public class LiftCommand {
    private LiftSubsystem liftSubsystem;
    private Hardware hw;
    private double power;

    private ElapsedTime elapsedTime;

    public LiftCommand(Hardware hw) {
        this.hw = hw;
        this.liftSubsystem = new LiftSubsystem(hw);
        hw.intake.setDirection(DcMotorSimple.Direction.REVERSE);
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




}




