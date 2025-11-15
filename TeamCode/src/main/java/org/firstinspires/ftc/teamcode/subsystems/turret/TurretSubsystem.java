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

    public double tanAdjustementBlue(double targetX, double targetY) {
        double currentX = mecanumCommand.getOdoX();
        double currentY = mecanumCommand.getOdoY();
        double currentHeading = (pinPointOdoSubsystem.getHeading());
        double normalizedCurrentHeading = currentHeading % (2 * Math.PI);

        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        double targetHeading = Math.atan2(deltaY, deltaX);

        double deltaHeading = targetHeading - normalizedCurrentHeading;
        return (deltaHeading > 180 ? 360 - deltaHeading : deltaHeading) + currentHeading;
    }
    public double tanAdjustementRed(double targetX, double targetY){
        double currentX = mecanumCommand.getOdoX();
        double currentY = mecanumCommand.getOdoY();
        double currentHeading = (pinPointOdoSubsystem.getHeading());
        double normalizedCurrentHeading = currentHeading % (2 * Math.PI);

        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        double targetHeading = Math.atan2(deltaY, deltaX);

        double deltaHeading = targetHeading - normalizedCurrentHeading;
        return -((deltaHeading > 180 ? 360 - deltaHeading : deltaHeading)) + currentHeading;
    }

}





