package org.firstinspires.ftc.teamcode.subsystems.turret;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.odometry.PinPointOdometrySubsystem;

public class TurretSubsystem {
    private Hardware hw;
    private MecanumCommand mecanumCommand;
    private PinPointOdometrySubsystem pinPointOdoSubsystem;

    double currentX;
    double currentY;
    double currentHeading;
    double targetX;
    double targetY;
    double offset = 0;
    double normalizedCurrentHeading;
    double deltaX;
    double deltaY;
    double deltaHeading;
    double targetHeading;
    double twoPi = 2*Math.PI;

    public TurretSubsystem(Hardware hw) {
        mecanumCommand = new MecanumCommand(hw);
        pinPointOdoSubsystem = new PinPointOdometrySubsystem(hw);


    }

    public void updateodo(){
        currentX = mecanumCommand.getOdoX();
        currentY = mecanumCommand.getOdoY();
        currentHeading = (pinPointOdoSubsystem.getHeading());
    }

    public double tanAdjustementBlue(double targetX, double targetY) {
        updateodo();
        targetHeading = Math.atan2(deltaY, deltaX);
        if ( currentHeading >= 0 ) {
            normalizedCurrentHeading = currentHeading % (2 * Math.PI);
            deltaX = targetX - currentX;
            deltaY = targetY - currentY;
            deltaHeading =  normalizedCurrentHeading - targetHeading;
            if (deltaHeading > Math.PI){
                return (twoPi - deltaHeading) + currentHeading - offset;
            } else {
               return  deltaHeading - currentHeading - offset;
            }
        } else {
            if (currentHeading > -targetHeading){
                return targetHeading;
            } else {
                normalizedCurrentHeading = -(Math.abs(currentHeading) % (2 * Math.PI));
                deltaX = targetX - currentX;
                deltaY = targetY - currentY;
                deltaHeading = normalizedCurrentHeading + targetHeading;
                if (deltaHeading > Math.PI) {
                    return (twoPi - deltaHeading) + currentHeading - offset;
                } else {
                    return deltaHeading - currentHeading - offset;
                }
            }
        }
    }

    public double tanAdjustementRed(double targetX, double targetY){
        normalizedCurrentHeading = currentHeading % (2 * Math.PI);
        deltaX = targetX - currentX;
        deltaY = targetY - currentY;
        targetHeading = Math.atan2(deltaY, deltaX);

        deltaHeading = targetHeading - normalizedCurrentHeading;
        return (deltaHeading > 180) ? (360 - deltaHeading) + currentHeading: currentHeading - deltaHeading;
    }
public void setTarget(){
 targetX = 36.0;
 targetY = 36.0;
 offset = -Math.PI/2;

}
    }





