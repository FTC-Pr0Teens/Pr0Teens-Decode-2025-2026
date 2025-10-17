package org.firstinspires.ftc.teamcode.subsystems.mecanum;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.odometry.PinPointOdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.pidcore.PIDCore;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Command wrapper for controlling a Mecanum drive system using
 * positional PID and field-oriented driving. This class consolidates
 * subsystem interactions and provides high-level movement commands.
 */
public class MecanumCommand {
    // PID controllers for x, y, and heading

        private MecanumSubsystem mecanumSubsystem;
        private PinPointOdometrySubsystem pinPointOdoSubsystem;
        private Hardware hw;

        public double xFinal, yFinal, thetaFinal;
        public double velocity;

        private double ex = 0;
        private double ey = 0;
        private double etheta = 0;

        public MecanumCommand(Hardware hw) {
            this.hw = hw;
            this.mecanumSubsystem = new MecanumSubsystem(hw);
            this.pinPointOdoSubsystem = new PinPointOdometrySubsystem(hw);
            xFinal = pinPointOdoSubsystem.getX();
            yFinal = pinPointOdoSubsystem.getY();
            thetaFinal = pinPointOdoSubsystem.getHeading();
            velocity = 0;
            turnOffInternalPID();
        }

        public void setConstants(double kpx, double kdx, double kix, double kpy, double kdy, double kiy, double kptheta, double kdtheta, double kitheta) {
            MecanumConstants.kpx = kpx;
            MecanumConstants.kdx = kdx;
            MecanumConstants.kix = kix;
            MecanumConstants.kpy = kpy;
            MecanumConstants.kdy = kdy;
            MecanumConstants.kiy = kiy;
            MecanumConstants.kptheta = kptheta;
            MecanumConstants.kdtheta = kdtheta;
            MecanumConstants.kitheta = kitheta;
            mecanumSubsystem.updatePIDConstants();
        }

        public void turnOffInternalPID() {
            mecanumSubsystem.turnOffInternalPID();
        }

    public void processPIDUsingPinpoint() {

        ex = mecanumSubsystem.globalXControllerOutputPositional(xFinal, pinPointOdoSubsystem.getX());
        ey = mecanumSubsystem.globalYControllerOutputPositional(yFinal, pinPointOdoSubsystem.getY());
        etheta = mecanumSubsystem.globalThetaControllerOutputPositional(thetaFinal, pinPointOdoSubsystem.getHeading());


        double max = Math.max(Math.abs(ex), Math.abs(ey));
        if (max > velocity) {
            double scalar = velocity / max;
            ex *= scalar;
            ey *= scalar;
            etheta *= scalar;
        }


        moveGlobalPartialPinPoint( ex, ey, etheta);
    }
    
        public void moveGlobalPartialPinPoint(double vertical, double horizontal, double rotational) {
            double angle = Math.PI / 2 - pinPointOdoSubsystem.getHeading();
            double localVertical = vertical * Math.cos(pinPointOdoSubsystem.getHeading()) - horizontal * Math.cos(angle);
            double localHorizontal = vertical * Math.sin(pinPointOdoSubsystem.getHeading()) + horizontal * Math.sin(angle);

            mecanumSubsystem.partialMove(localVertical, localHorizontal, rotational);

        }

        public void resetPinPointOdometry() {
            pinPointOdoSubsystem.reset();
        }

        public boolean moveToPos(double x, double y, double theta) {
            setFinalPosition(0.67, x, y, theta);
            return!(positionNotReachedYet());
        }

        public void setFinalPosition(double velocity, double x, double y, double theta) {
            this.xFinal = x;
            this.yFinal = y;
            this.thetaFinal = theta;
            this.velocity = velocity;
        }

        public boolean positionNotReachedYet() {
            return !(isXReached() && isYReached() && isThetaReached());
        }

        public double getXDifferencePinPoint() {
            return Math.abs(this.xFinal - pinPointOdoSubsystem.getX());
        }

        public double getYDifferencePinPoint() {
            return Math.abs(this.yFinal - pinPointOdoSubsystem.getY());
        }

        public double getThetaDifferencePinPoint() {
            return Math.abs(this.thetaFinal - pinPointOdoSubsystem.getHeading());
        }

        public boolean isYReached() {
            return getYDifferencePinPoint() < 2.5;
        }

        public boolean isXReached() {
            return getXDifferencePinPoint() < 2.5;
        }

        public boolean isThetaReached() {
            return getThetaDifferencePinPoint() < 0.07;
        }

        public double getOdoX(){
            return pinPointOdoSubsystem.getX();
        }

        public double getOdoY(){
            return pinPointOdoSubsystem.getY();
        }

        public double getOdoHeading(){
            return pinPointOdoSubsystem.getHeading();
        }

        public boolean isThetaPassed(){
            return getThetaDifferencePinPoint() < 0.22;
        }

        public boolean isXPassed(){
            return getXDifferencePinPoint() < 10;
        }

        public boolean isYPassed(){
            return getYDifferencePinPoint() < 10;
        }

        public double fieldOrientedMove(double vertical, double horizontal, double rotational) {
            mecanumSubsystem.fieldOrientedMove(vertical, horizontal, rotational, pinPointOdoSubsystem.getHeading());
            return pinPointOdoSubsystem.getHeading();
        }

        public void motorProcess() {

            mecanumSubsystem.motorProcessNoEncoder();
        }


        public void deadReckoning() {
            pinPointOdoSubsystem.deadReckoning();
        }

        public void stop(){
            mecanumSubsystem.setPowers(0,0,0,0);
        }

        public void processOdometry(){
            // Use the new unified update method
            pinPointOdoSubsystem.updateOdometry();
        }

        // Debug helper
        public void debugPID(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
            telemetry.addData("PID ex (global)", ex);
            telemetry.addData("PID ey (global)", ey);
            telemetry.addData("PID etheta", etheta);
        }
    }








