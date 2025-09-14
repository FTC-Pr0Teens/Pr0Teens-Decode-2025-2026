package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;

public class Hardware {


    //singleton
    private static Hardware instance;



    public final DcMotorEx lf;
    public final DcMotorEx rf;
    public final DcMotorEx lb;
    public final DcMotorEx rb;

    // Servos

//    public final Servo lArm;
//    public final Servo rArm;
//
//    public final Servo lLinkage;
//    public final Servo rLinkage;

    // Odometry
    public final GoBildaPinpointDriver pinPointOdo;

    public Hardware(HardwareMap hwMap){




        this.rf = hwMap.get(DcMotorEx.class, Specifications.FTRT_MOTOR); //rightforward
        this.lf = hwMap.get(DcMotorEx.class, Specifications.FTLF_MOTOR); //leftforward
        this.lb = hwMap.get(DcMotorEx.class, Specifications.BKLF_MOTOR); //leftback
        this.rb = hwMap.get(DcMotorEx.class, Specifications.BKRT_MOTOR); //rightback

//        this.lArm = hwMap.get(Servo.class, Specifications.LEFT_OUTPUT_ARM);
//        this.rArm = hwMap.get(Servo.class, Specifications.RIGHT_OUTPUT_ARM);
//
//        this.lLinkage = hwMap.get(Servo.class, Specifications.EXTENSION_ARM_LEFT);
//        this.rLinkage = hwMap.get(Servo.class, Specifications.EXTENSION_ARM_RIGHT);

        this.pinPointOdo = hwMap.get(GoBildaPinpointDriver.class, Specifications.PIN_POINT_ODOMETRY);
    }

    public static Hardware getInstance(HardwareMap hwMap) {
        if (instance == null) {
            instance = new Hardware(hwMap);
        }
        return instance;
    }
}
