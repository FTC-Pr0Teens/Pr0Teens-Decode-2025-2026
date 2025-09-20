package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;

public class Hardware {

    public static final String EXTENSION_MOTOR_MAIN = "lift1";
    public static final String EXTENSION_MOTOR_AUX1 = "lift2";
    public static final String EXTENSION_MOTOR_AUX2 = "lift3";
    //singleton
    private static Hardware instance;

    //    // Motors

    private Limelight3A limelight;
    public final DcMotorEx lf;
    public final DcMotorEx rf;
    public final DcMotorEx lb;
    public final DcMotorEx rb;

    // Servos
    public final DcMotorEx lout;
    public final DcMotorEx rout;

    public final CRServo turret;
    public final CRServo intake;
    public final Servo push;
    private final Servo shooteraim;

    // Odometry
    public final GoBildaPinpointDriver pinPointOdo;

    public Hardware(HardwareMap hwMap) {

//

        this.rf = hwMap.get(DcMotorEx.class, Specifications.FTRT_MOTOR); //rightforward
        this.lf = hwMap.get(DcMotorEx.class, Specifications.FTLF_MOTOR); //leftforward
        this.lb = hwMap.get(DcMotorEx.class, Specifications.BKLF_MOTOR); //leftback
        this.rb = hwMap.get(DcMotorEx.class, Specifications.BKRT_MOTOR); //rightback

        this.lout = hwMap.get(DcMotorEx.class, Specifications.LOUT); //leftback
        this.rout = hwMap.get(DcMotorEx.class, Specifications.ROUT);

        this.intake = hwMap.get(CRServo.class, Specifications.INTAKE);
        this.turret = hwMap.get(CRServo.class, Specifications.TURRET);
        this.push = hwMap.get(Servo.class, Specifications.push);
        this.shooteraim = hwMap.get(Servo.class, Specifications.shooteraim);




        this.pinPointOdo = hwMap.get(GoBildaPinpointDriver.class, Specifications.PIN_POINT_ODOMETRY);

    }

    public static Hardware getInstance(HardwareMap hwMap) {
        if (instance == null) {
            instance = new Hardware(hwMap);
        }
        return instance;
    }
}


