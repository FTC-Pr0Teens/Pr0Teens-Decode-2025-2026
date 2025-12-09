package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.odometry.PinPointOdometryConstants;
import org.firstinspires.ftc.teamcode.subsystems.odometry.PinPointOdometrySubsystem;

@TeleOp
public class Reading  extends LinearOpMode {
    PinPointOdometrySubsystem odo;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hw = Hardware.getInstance(hardwareMap);
        odo = new PinPointOdometrySubsystem(hw);
        waitForStart();
        while (opModeIsActive()){
            odo.processOdometry();
            telemetry.addData("x", odo.getX());
            telemetry.addData("y: ", odo.getY());
            telemetry.addData("heading: ", odo.getHeading());
            telemetry.update();
        }
    }
}
