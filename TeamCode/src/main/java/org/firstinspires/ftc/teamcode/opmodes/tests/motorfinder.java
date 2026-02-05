/**
 * ===============================================================================
 * FIRST TECH CHALLENGE - MOTOR DIAGNOSTIC AND TESTING UTILITY
 * ===============================================================================
 *
 * FILE: motorfinder.java
 * TEAM: Pr0Teens (FTC Team)
 * SEASON: 2025-2026
 *
 * DESCRIPTION:
 * This is a diagnostic TeleOp program designed for hardware verification and
 * motor identification during robot setup and troubleshooting. It allows drivers
 * to test each drivetrain motor individually to verify:
 *   - Motor port assignments and wiring
 *   - Motor direction configurations
 *   - Physical motor operation and responsiveness
 *   - Proper hardware map naming conventions
 *
 * USE CASES:
 * - Initial robot assembly and wiring verification
 * - Troubleshooting motor issues during competition
 * - Training new drivers on individual motor control
 * - Verifying motor replacement or rewiring
 * - Testing motor direction settings before autonomous/teleop
 *
 * CONTROL SCHEME (Gamepad 1):
 * - A Button: Activate LEFT FRONT motor (lf) at 50% power
 * - B Button: Activate LEFT BACK motor (lb) at 50% power
 * - X Button: Activate RIGHT FRONT motor (rf) at 50% power
 * - Y Button: Activate RIGHT BACK motor (rb) at 50% power
 *
 * MOTOR DIRECTIONS CONFIGURED:
 * - Left side motors (lf, lb):  REVERSE direction
 * - Right side motors (rf, rb): FORWARD direction
 *
 * TESTING PROCEDURE:
 * 1. Place robot on blocks or ensure safe testing environment
 * 2. Start this OpMode from driver station
 * 3. Press each button (A, B, X, Y) individually
 * 4. Verify corresponding wheel rotates in expected direction
 * 5. Confirm motor power and response are satisfactory
 *
 * NOTE: Motors run at 50% power for safety during testing.
 * ===============================================================================
 */
package org.firstinspires.ftc.teamcode.opmodes.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp(name = "motor finder")
public class motorfinder extends LinearOpMode {

        private Hardware hw;
        private DcMotor leftFront;
        private DcMotor leftRear;
        private DcMotor rightFront;
        private DcMotor rightRear;

        public void runOpMode() throws InterruptedException {
            hw = Hardware.getInstance(hardwareMap);
            leftFront = hardwareMap.get(DcMotor.class, "lf");
            rightFront = hardwareMap.get(DcMotor.class, "rf");
            leftRear = hardwareMap.get(DcMotor.class, "lb");
            rightRear = hardwareMap.get(DcMotor.class, "rb");

            hw.lb.setDirection(DcMotorSimple.Direction.REVERSE);
            hw.lf.setDirection(DcMotorSimple.Direction.REVERSE);
            hw.rf.setDirection(DcMotorSimple.Direction.FORWARD);
            hw.rb.setDirection(DcMotorSimple.Direction.FORWARD);


            waitForStart();


            while (opModeIsActive()) {
                if (gamepad1.a) {
                    hw.lf.setPower(0.5); //
                } else {
                    hw.lf.setPower(0);
                }
                if (gamepad1.b) {
                    hw.lb.setPower(0.5); //
                } else {
                    hw.lb.setPower(0);
                }
                if (gamepad1.x) {
                    hw.rf.setPower(0.5); //
                } else {
                    hw.rf.setPower(0);
                }
                if (gamepad1.y) {
                    hw.rb.setPower(0.5); //
                } else {
                    hw.rb.setPower(0);
                }
            }
        }
}
