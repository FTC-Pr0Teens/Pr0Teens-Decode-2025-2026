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
    private double targetRPM1;

    double DEFAULT_RPM = 2800;
    double DEFAULT_RPM1 = 4000;

    public OuttakeCommand(Hardware hw) {
        this.hw = hw;
        this.outtakeSubsystem = new OuttakeSubsystem(hw);
        this.shooter = hw.shooter;
        this.targetRPM = DEFAULT_RPM;
        this.targetRPM1 = DEFAULT_RPM1;
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // hw.shooter.setVelocityPIDFCoefficients(1.3, 0.0,0.002,0);
    }

    //returns whether or not we have reached the correctRPM
    public boolean isRPMReached(double currentRPM) {
        return Math.abs(targetRPM - currentRPM) < 200;
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

    public void setMaxRPM(int maxRPM){
        targetRPM = maxRPM;
    }

    public boolean spinupmid(){
        double currentRPM1 = hw.shooter.getVelocity() * 60.0 / 28.0;
        double targetTPS1 = targetRPM1 * 28.0 / 60.0;

        hw.shooter.setVelocity(targetTPS1);

        return isRPMReached(currentRPM1);
    }

}







