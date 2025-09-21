//package org.firstinspires.ftc.teamcode.opmodes.tests.utiltest;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Hardware;
//import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommand;
//import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
//
//@TeleOp(name = "chaewon teleop")
//public class felixteleop extends LinearOpMode {
//    private MecanumCommand mecanumCommand;
//    private LiftCommand liftCommand;
//    private ElapsedTime timer;
//    private ElapsedTime resetTimer;
//    private Hardware hw;
//
//    // Auto-aim state
//    private boolean autoAimEnabled = false;
//    private boolean lastAutoAimButton = false;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        hw = Hardware.getInstance(hardwareMap);
//        mecanumCommand = new MecanumCommand(hw);
//        liftCommand = new LiftCommand(hw);
//        resetTimer = new ElapsedTime();
//        timer = new ElapsedTime();
//
//        waitForStart();
//
//        // Loop while OpMode is running
//        while (opModeIsActive()) {
//
//            handleDriveControls();
//
//            handleLiftControls();
//
//
//            handleAutoAimControls();
//
//
//            handleShooterControls();
//
////            liftCommand.update();
//            mecanumCommand.deadReckoning();
//            mecanumCommand.motorProcess();
//
//            processTelemetry();
//
//            if (gamepad1.start) {
//                mecanumCommand.resetPinPointOdometry();
//            }
//        }
//    }
//
//    private void handleDriveControls() {
//        if (!autoAimEnabled) {
//            // Normal field-oriented driving
//            double vertical = -gamepad1.left_stick_y;
//            double horizontal = gamepad1.left_stick_x;
//            double rotational = gamepad1.right_stick_x;
//
//            mecanumCommand.fieldOrientedMove(vertical, horizontal, rotational);
//        } else {
//
//            double vertical = -gamepad1.left_stick_y;
//            double horizontal = gamepad1.left_stick_x;
//
//            mecanumCommand.autoAimMove(vertical, horizontal);
//        }
//    }
//
//    private void handleLiftControls() {
//        // Intake controls
//        if (gamepad2.right_bumper) {
//            liftCommand.handleIntake();
//        } else if (gamepad2.left_bumper) {
//            liftCommand.stopintake();
//        }
//
//
//        if (gamepad2.y && !(gamepad2.right_trigger > 0.1)) {
//            liftCommand.out();
//        } else if (!gamepad2.dpad_up) {
//            liftCommand.outstop();
//        }
//
//
//        if (gamepad2.a) {
//            liftCommand.push();
//        } else if (gamepad2.b) {
//            liftCommand.pull();
//        }
//    }
//
//    private void handleAutoAimControls() {
//
//        boolean currentAutoAimButton = gamepad1.x;
//        if (currentAutoAimButton && !lastAutoAimButton) {
//            autoAimEnabled = !autoAimEnabled;
//        }
//        lastAutoAimButton = currentAutoAimButton;
//    }
//
//    private void handleShooterControls() {
//
//        if (gamepad2.right_trigger > 0.1) {
//
//            double shooterSpeed = gamepad2.right_trigger;
//            liftCommand.setShooterVelocity(shooterSpeed);
//        } else if (gamepad2.x) {
//
//            liftCommand.setShooterVelocity(1.0);
//        } else if (gamepad2.left_trigger > 0.1) {
//
//            liftCommand.setShooterVelocity(0.7);
//        } else {
//            // Stop shooter
//            liftCommand.outstop();
//        }
//
//        // Feed shooter (replaces old turret control)
//        if (gamepad2.dpad_up) {
//            liftCommand.push();
//        } else if (gamepad2.dpad_down) {
//            // Reverse feed for unjamming
//            liftCommand.turret(); // Use original turret method for reverse
//        } else {
//            liftCommand.push();
//        }
//
//
//        if (gamepad2.start && liftCommand.isShooterAtSpeed()) {
//            liftCommand.autoFire();
//        }
//    }
//
//    public void processTelemetry() {
//        telemetry.addData("Auto-Aim Enabled: ", autoAimEnabled);
//
//    }
//
//}