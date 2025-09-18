package org.firstinspires.ftc.teamcode.opmodes.tests.utiltest;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "out")
public class outtest extends  LinearOpMode {
    private DcMotor leftLaunch;
    private DcMotor rightLaunch;


    public void runOpMode() throws InterruptedException {
        // Set motor directions

        leftLaunch = hardwareMap.get(DcMotor.class, "leftLaunch");
        rightLaunch = hardwareMap.get(DcMotor.class, "rightLaunch");
        leftLaunch.setDirection(DcMotor.Direction.REVERSE);
        rightLaunch.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a){
                leftLaunch.setPower(-0.8);
                rightLaunch.setPower(0.8);
            }
            if(gamepad1.b){
                leftLaunch.setPower(0);
                rightLaunch.setPower(0);
            }
        }
    }
}