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

    double twoPi = 2 * Math.PI;

    private Alliance alliance;

    public enum Alliance {
        RED,
        BLUE
    }

    public TurretSubsystem(Hardware hw, String alliance) {
        mecanumCommand = new MecanumCommand(hw);
        pinPointOdoSubsystem = new PinPointOdometrySubsystem(hw);
        if (alliance == "BLUE") {
            this.alliance = Alliance.BLUE;
        } else {
            this.alliance = Alliance.RED;
        }

    }

    public void updateOdo() {
        currentX = mecanumCommand.getOdoX();
        currentY = mecanumCommand.getOdoY();
        currentHeading = mecanumCommand.getOdoHeading();
    }

    //lets make facing forward/North 0 theta
    public double tanAdjustment() {
        updateOdo();

        // Calculate vector to target
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;

        // Calculate desired heading to face target
        double targetHeading = Math.atan2(deltaY, deltaX);
        double desiredHeading = targetHeading + offset;
        double normalizedCurrentHeading = currentHeading % (twoPi);
        if (currentHeading >= 0) {
            double deltaHeading = Math.abs(normalizedCurrentHeading - targetHeading);
            if (deltaHeading > Math.PI) {
                return (twoPi - deltaHeading) + currentHeading;
            } else {
                return currentHeading - deltaHeading;
            }
        } else {
            double deltaHeading = normalizedCurrentHeading - targetHeading;
            if (deltaHeading > Math.PI) {
                return (twoPi - deltaHeading) - currentHeading;
            } else {
                return currentHeading + deltaHeading;
            }
        }

    }

    //assuming we start at the center of the field
    // change these accordingly
    public void setTargetCentered() {
        if (alliance == Alliance.BLUE){
            targetX = 600.0;
            targetY = 600.0;
        } else if (alliance == Alliance.RED){
            targetX = 600.0;
            targetY = -600.0;
        }
    }
}





