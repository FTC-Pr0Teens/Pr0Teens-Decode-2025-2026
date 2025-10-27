package org.firstinspires.ftc.teamcode.util;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.camera.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Hardware;
//import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommand;
//import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;


@TeleOp(name = "Limelight", group = "TeleOp")
public class SampleTeleOpMode extends LinearOpMode {
    private Hardware hw;
    private LimelightSubsystem limelightsub;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = Hardware.getInstance(hardwareMap);
        limelightsub = new LimelightSubsystem(hw, telemetry);

        // Wait for start button to be pressed
        waitForStart();

        // Loop while OpMode is running
        while (opModeIsActive()) {
            limelightsub.telemetryLimelight(telemetry);
        }

    }
}




