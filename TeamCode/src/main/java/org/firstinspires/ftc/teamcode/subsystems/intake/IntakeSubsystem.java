package org.firstinspires.ftc.teamcode.subsystems.intake;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware;
public class IntakeSubsystem {

    private final Hardware hw;

    public IntakeSubsystem(Hardware hw) {
        this.hw = hw;
        hw.intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void intake() {
        hw.intake.setPower(1.0);

    }



    public void stopintake() {
        hw.intake.setPower(0.0);

    }

//    public void turretTurn() {
//        hw.turn.setPosition(0.0);
//
//    }
//    public void turretTurn2() {
//        hw.turn.setPosition(0.18);
//
//    }
//    public void turretTurn3() {
//        hw.turn.setPosition(0.36);

    //    }
    public void push() {
        hw.pusher.setPosition(0.3);

    }
//    public void rainbetIntake(){
//        hw.sorter.setPower(0.67);
//    }
//    public void stopTurn(){
//        hw.sorter.setPower(0.0);
//    }


    public void shooter(double targetVelocity) {
        hw.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.shooter.setVelocityPIDFCoefficients(0.002, 0.0001, 0.0001, 12.0);
        hw.shooter.setVelocity(targetVelocity);

    }
    public void shooterstop() {
        hw.shooter.setVelocity(0);

    }
    public void pull() {
        hw.pusher.setPosition(0.0);

    }
}
