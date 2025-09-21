package org.firstinspires.ftc.teamcode.subsystems.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;

public class OuttakeCommand {

    private Hardware hw;
    private OuttakeSubsystem outtakeSubsystem;


    public OuttakeCommand(Hardware hw) {
        this.hw = hw;


        this.outtakeSubsystem = new OuttakeSubsystem();


    }

    public double shoot() {


        return 0;
    }
}








