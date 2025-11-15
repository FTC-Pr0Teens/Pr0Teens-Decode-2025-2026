package org.firstinspires.ftc.teamcode.subsystems.outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;

public class OuttakeCommand {

    private Hardware hw;
    private OuttakeSubsystem outtakeSubsystem;

    private DcMotorEx shooter;

    private double targetRPM;

    double DEFAULT_RPM = 5000;

    public OuttakeCommand(Hardware hw) {
        this.hw = hw;
        this.outtakeSubsystem = new OuttakeSubsystem(hw);
        this.shooter = hw.shooter;
        this.targetRPM = DEFAULT_RPM;
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //returns whether or not we have reached the correctRPM
    public boolean isRPMReached(double currentRPM) {
        return Math.abs(targetRPM - currentRPM) > 200;
    }

    public boolean spinup(){
        double currentRPM = hw.shooter.getVelocity() * 60.0 / 28.0;
        double targetTPS = targetRPM * 28.0 / 60.0;

        hw.shooter.setVelocity(targetTPS);

        return isRPMReached(currentRPM);
    }
    public void stopShooter(){
        hw.shooter.setVelocity(0);
    }
}







