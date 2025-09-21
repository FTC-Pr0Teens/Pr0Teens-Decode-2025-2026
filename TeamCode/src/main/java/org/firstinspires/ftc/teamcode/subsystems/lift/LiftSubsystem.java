//package org.firstinspires.ftc.teamcode.subsystems.lift;
//
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.Hardware;
//import org.firstinspires.ftc.teamcode.util.pidcore.PIDCore;
//
//class LiftSubsystem {
//    private final Hardware hw;
//
//
//    public LiftSubsystem(Hardware hw) {
//        this.hw = hw;
//
//    }
//
//    public void intake() {
//        hw.intake.setPower(1.0);
//
//    }
//
//    public void outtake() {
//        hw.lout.setPower(-0.8);
//        hw.rout.setPower(0.8);
//    }
//    public void outtakestop() {
//        hw.lout.setPower(0.0);
//        hw.rout.setPower(0.0);
//    }
//
//
//    public void stopintake() {
//        hw.intake.setPower(0.0);
//
//    }
//
//    public void turret() {
//        hw.turret.setPower(1.0);
//
//    }
//    public void turretstop() {
//        hw.turret.setPower(0.0);
//
//    }
//    public void push(){
//        hw.push.setDirection(Servo.Direction.REVERSE);
//        hw.push.setPosition(0.6);
//    }
//    public void pull(){
//        hw.push.setPosition(0.27);
//    }
//
//
//
//}
//
//
//
//
//
