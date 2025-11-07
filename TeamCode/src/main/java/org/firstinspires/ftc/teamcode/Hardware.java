package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;

public class Hardware {

    //singleton
    private static Hardware instance;

    // Motors
//  public Limelight3A limelight;
    public final DcMotorEx intake;
    public final Servo sorter;
    public final Servo pusher;
    public final CRServo turret;
    public final DcMotorEx lf;
    public final DcMotorEx rf;
    public final DcMotorEx lb;
    public final DcMotorEx rb;
    public final DcMotorEx shooter;
    // Odometry
    public final GoBildaPinpointDriver pinPointOdo;
    //public final ColorSensor colourSensor;

    public Hardware(HardwareMap hwMap){
        this.rf = hwMap.get(DcMotorEx.class, Specifications.FTRT_MOTOR); //rightforward
        this.lf = hwMap.get(DcMotorEx.class, Specifications.FTLF_MOTOR); //leftforward
        this.lb = hwMap.get(DcMotorEx.class, Specifications.BKLF_MOTOR); //leftback
        this.rb = hwMap.get(DcMotorEx.class, Specifications.BKRT_MOTOR);//rightback
        this.pusher = hwMap.get(Servo.class, Specifications.PUSHER);
//        this.limelight = hwMap.get(Limelight3A.class, Specifications.LIME_LIGHT);
        this.intake = hwMap.get(DcMotorEx.class, Specifications.INTAKE);
        this.shooter = hwMap.get(DcMotorEx.class, Specifications.SHOOTER);
        this.sorter = hwMap.get(Servo.class, Specifications.SORTER);
        this.turret = hwMap.get(CRServo.class, Specifications.TURRET);

        this.pinPointOdo = hwMap.get(GoBildaPinpointDriver.class, Specifications.PIN_POINT_ODOMETRY);
        //this.colourSensor = hwMap.get(ColorSensor.class, Specifications.colourSensor);
    }

    public static Hardware getInstance(HardwareMap hwMap) {
        if (instance == null) {
            instance = new Hardware(hwMap);
        }
        return instance;
    }
}
