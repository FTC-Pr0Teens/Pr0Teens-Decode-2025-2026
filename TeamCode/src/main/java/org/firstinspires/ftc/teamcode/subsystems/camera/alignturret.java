package org.firstinspires.ftc.teamcode.subsystems.camera;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class alignturret {

    private final Hardware hw;
    private final Limelight3A limelight;

    private static final double kP = 0.02;
    private static final double kI = 0.0;
    private static final double kD = 0.001;
    private static final double MAX_POWER = 0.4;

    private double previousError = 0;
    private double integral = 0;

    private double initialHeading = 0;
    private double currentHeading = 0;

    private boolean active = false;

    public alignturret(Hardware hw, Limelight3A limelight) {
        this.hw = hw;
        this.limelight = limelight;
    }

    public void start() {
        limelight.pipelineSwitch(6);
        limelight.start();




        active = true;
    }

    public void stop() {
        active = false;
        hw.turret.setPower(0);
        limelight.stop();

        integral = 0;
        previousError = 0;
    }

    public void update() {
        if (!active) return;

        LLResult result = limelight.getLatestResult();



        double headingOffset = currentHeading - initialHeading;


        if (headingOffset > 180) headingOffset -= 360;
        if (headingOffset < -180) headingOffset += 360;

        double error;

        if (result != null && result.isValid()) {
            double tx = result.getTx();
            error = tx - headingOffset;
        } else {
            error = -headingOffset;
        }

        // PID control
        integral += error;
        integral = Math.max(-50, Math.min(50, integral));

        double derivative = error - previousError;
        previousError = error;

        double output = (kP * error) + (kI * integral) + (kD * derivative);
        output = Math.max(-MAX_POWER, Math.min(MAX_POWER, output));

        // Apply power (invert if needed based on turret motor direction)
        hw.turret.setPower(-output);
    }

    public boolean isActive() {
        return active;
    }
}