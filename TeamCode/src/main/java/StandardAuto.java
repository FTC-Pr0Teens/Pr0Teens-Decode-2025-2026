import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="StandardAuto", group = "Auto")
public class StandardAuto extends LinearOpMode {

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    @Override
    public void runOpMode() throws InterruptedException{
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        waitForStart();
        if(!opModeIsActive()) return;

        drive(0.5,0.5, 500);
        drive(-0.5,-0.5, 500);
        //change 500 according to distance
    }

    private void drive(double leftPower, double rightPower, double ms) {
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        waitTime(ms);

        stopAll();
    }
    public void waitTime(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        // Wait while OpMode is still active and time hasn't elapsed
        while (timer.seconds() < seconds) {
            // Optionally do telemetry updates
            telemetry.addData("Waiting", "%.1f / %.1f", timer.seconds(), seconds);
            telemetry.update();
        }
    }
    private void stopAll() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
