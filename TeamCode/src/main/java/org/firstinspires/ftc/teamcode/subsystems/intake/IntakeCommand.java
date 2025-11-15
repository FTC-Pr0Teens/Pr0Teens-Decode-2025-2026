//package org.firstinspires.ftc.teamcode.subsystems.intake;
//
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Hardware;
//import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeSubsystem;
//
//public class IntakeCommand {
//    private IntakeSubsystem intakeSubsystem;
//    private OuttakeSubsystem outtakeSubsystem;
//    private Hardware hw;
//    private double power;
//
//    private ElapsedTime elapsedTime;
//
//    public IntakeCommand(Hardware hw) {
//        this.hw = hw;
//        this.intakeSubsystem = new IntakeSubsystem(hw);
//        this.outtakeSubsystem = new OuttakeSubsystem(hw);
//        hw.intake.setDirection(DcMotorSimple.Direction.REVERSE);
//        hw.shooter.setDirection(DcMotorSimple.Direction.REVERSE);
//        elapsedTime = new ElapsedTime();
//        this.power = 0;
//
//
//        elapsedTime = new ElapsedTime();
//    }
//
//    public void stopintake() {
//        intakeSubsystem.stopintake();
//    }
//
//    public void handleIntake() {
//        intakeSubsystem.intake();
//    }
//
//    //    public void turn(){ intakeSubsystem.turretTurn();}
////    public void turn2(){
////        intakeSubsystem.turretTurn2();
////    }
////    public void turn3(){
////        intakeSubsystem.turretTurn3();
////    }
//    public void shoot(){
//        double targetRPM = 5600;
//        double currentRPM = hw.shooter.getVelocity();
//        double output = outtakeSubsystem.outputPositional(targetRPM, currentRPM);
//        hw.shooter.setPower(output);
//
//    }
//    public void push(){
//        intakeSubsystem.push();
//    }
//    public void pull(){
//        intakeSubsystem.pull();
//    }
//    public void shootstop(){
//        intakeSubsystem.shooterstop();
//    }
////    public void rainbet(){
////        intakeSubsystem.rainbetIntake();
////    }
////    public void rainbetStop(){
////        intakeSubsystem.stopTurn();
////    }
//
//
//
//
//
//
//}
//
//
//
