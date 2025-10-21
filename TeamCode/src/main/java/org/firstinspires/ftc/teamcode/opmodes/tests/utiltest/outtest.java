package org.firstinspires.ftc.teamcode.opmodes.tests.utiltest;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.pidcore.PIDCore;


@TeleOp(name = "out")
public class outtest extends LinearOpMode {
    private ElapsedTime timer;
    private DcMotorEx shooter;
    private OuttakeSubsystem outtakeSubsystem;
    private Hardware hw;

    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        outtakeSubsystem = new OuttakeSubsystem(hw);

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        timer.reset();

        double targetVel = 67;

        while (opModeIsActive()) {
            double currentVel = shooter.getVelocity();
            double power = outtakeSubsystem.outputPositional(targetVel, currentVel);
            shooter.setPower(power);



            telemetry.addData("Target Velocity", targetVel);
            telemetry.addData("Actual Velocity", currentVel);
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }

}


