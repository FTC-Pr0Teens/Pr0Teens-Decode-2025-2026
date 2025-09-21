package org.firstinspires.ftc.teamcode.opmodes.tests.utiltest;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
@TeleOp(name = "volts")
public class voltagetest extends LinearOpMode {

    @Override
    public void runOpMode() {
        VoltageSensor controlHubVoltage = hardwareMap.get(VoltageSensor.class, "Control Hub");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Control Hub Battery Voltage", "%.2f V", controlHubVoltage.getVoltage());
            telemetry.update();
        }
    }
}
