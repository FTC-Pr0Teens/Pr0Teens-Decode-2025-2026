
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="MecanumAuto", group = "Auto")
public class MecanumAuto extends LinearOpMode{
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
            public void runOpMode() throws InterruptedException{
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        waitForStart();
        if (!opModeIsActive()) return;

    }
    private void forward (double power, long ms){
        drive(power, power, power, power);
        waitTime(ms);
    }
    private void backward (double power, long ms){
        drive(-power, -power, -power, -power);
        waitTime(ms);
    }
    private void pivotLeft (double power, long ms){
        drive(-power, power, power, power);
        waitTime(ms);
    }
    private void pivotRight (double power, long ms){
        drive(power, -power, -power, power);
        waitTime(ms);
    }
    private void right (double power, long ms){
        drive(power, -power, power, -power);
        waitTime(ms);
    }
    private void left (double power, long ms){
        drive(-power, power, -power, power);
        waitTime(ms);
    }

    private void drive(double fl, double fr, double bl, double br) {

        ElapsedTime t = new ElapsedTime();
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    
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



}


