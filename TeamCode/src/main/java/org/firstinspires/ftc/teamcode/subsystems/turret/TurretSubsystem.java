package org.firstinspires.ftc.teamcode.subsystems.turret;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.odometry.PinPointOdometrySubsystem;

public class TurretSubsystem {
    private Hardware hw;
    private MecanumCommand mecanumCommand;
    private PinPointOdometrySubsystem pinPointOdoSubsystem;


    public TurretSubsystem(Hardware hw) {
        mecanumCommand = new MecanumCommand(hw);
        pinPointOdoSubsystem = new PinPointOdometrySubsystem(hw);


    }

    public void tanAdjustement(double targetX, double targetY) {
        double currentX = mecanumCommand.getOdoX();
        double currentY = mecanumCommand.getOdoY();


        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;

        double currentHeading = normalizeAngle(pinPointOdoSubsystem.getHeading());
        double targetHeading = Math.atan2(deltaY, deltaX);

        double error = angleWrap(targetHeading - currentHeading);


        double correction = error * 0.5;

        mecanumCommand.setRotationPower(correction);
    }

    public double normalizeAngle(double angle) {

        double twoPi = 2 * Math.PI;
        return ((angle % twoPi) + twoPi) % twoPi;


    }


    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}



