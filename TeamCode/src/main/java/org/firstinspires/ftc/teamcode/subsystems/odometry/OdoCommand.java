package org.firstinspires.ftc.teamcode.subsystems.odometry;

import org.firstinspires.ftc.teamcode.Hardware;

public class OdoCommand {
    private PinPointOdometrySubsystem pinPointOdometrySubsystem;
    private Hardware hw;


    public OdoCommand(Hardware hw) {
        this.hw = hw;
        this.pinPointOdometrySubsystem = new PinPointOdometrySubsystem(hw);



    }
}
