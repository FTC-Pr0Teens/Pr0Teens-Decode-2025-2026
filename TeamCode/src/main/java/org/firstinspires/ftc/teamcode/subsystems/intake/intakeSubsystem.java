//package org.firstinspires.ftc.teamcode.subsystems.intake;
//
//
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.Hardware;
//import org.firstinspires.ftc.teamcode.util.pidcore.PIDCore;
//
//class IntakeSubsystem {
//    private final Hardware hw;
//
//
//    public IntakeSubsystem(Hardware hw) {
//        this.hw = hw;
//
//
//    }
//
//    public void intake() {
//        hw.intake.setPower(1.0);
//
//    }
//
//
//
//    public void stopintake() {
//        hw.intake.setPower(0.0);
//
//    }
//    public void turretTurn() {
//        hw.sorter.setPosition(0.5);
//    }
//    public void turret2() {
//        hw.sorter.setPosition(0.0);
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
