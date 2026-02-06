/**
 * ===============================================================================
 * FIRST TECH CHALLENGE - INTAKE SUBSYSTEM
 * ===============================================================================
 *
 * FILE: IntakeSubsystem.java
 * PACKAGE: org.firstinspires.ftc.teamcode.subsystems.intake
 * TEAM: Pr0Teens (FTC Team)
 * SEASON: 2025-2026
 *
 * DESCRIPTION:
 * This subsystem encapsulates control of the robot's intake mechanism, providing
 * a simplified interface for game element collection and manipulation. The class
 * manages multiple related mechanisms:
 *   - Intake motor control for collecting game elements
 *   - Pusher servo mechanism for specimen scoring
 *   - Shooter wheel velocity control with custom PIDF tuning
 *
 * DESIGN PATTERN:
 * This class follows the subsystem pattern, abstracting hardware complexity and
 * providing high-level methods that can be called from autonomous and teleop programs.
 * The subsystem is initialized with a Hardware instance and maintains references
 * to all controlled components.
 *
 * KEY FEATURES:
 * - Simplified intake control (on/off via single method calls)
 * - Pusher actuation for scoring operations
 * - Velocity-controlled shooter with custom PIDF coefficients
 * - Stateless design - no internal state tracking required
 *
 * HARDWARE COMPONENTS CONTROLLED:
 * - intake: DC motor for game element collection (reversed direction)
 * - pusher: Servo for specimen pushing mechanism
 * - shooter: DC motor with encoder for velocity-controlled shooting
 *
 * LEGACY/DISABLED FEATURES:
 * Several methods related to turret control and rainbow belt intake are
 * commented out, suggesting evolution of robot design over the season.
 * These are preserved for reference and potential future use.
 *
 * NOTE: This is the correct package location for the intake subsystem.
 * A duplicate version exists in subsystems.cameras package, likely due to
 * historical reorganization.
 * ===============================================================================
 */
package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware;
public class IntakeSubsystem {

    private final Hardware hw;

    public IntakeSubsystem(Hardware hw) {
        this.hw = hw;
        hw.intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void intake() {
        hw.intake.setPower(1.0);

    }



    public void stopintake() {
        hw.intake.setPower(0.0);

    }

//    public void turretTurn() {
//        hw.turn.setPosition(0.0);
//
//    }
//    public void turretTurn2() {
//        hw.turn.setPosition(0.18);
//
//    }
//    public void turretTurn3() {
//        hw.turn.setPosition(0.36);

    //    }
    public void push() {
        hw.pusher.setPosition(0.3);

    }
//    public void rainbetIntake(){
//        hw.sorter.setPower(0.67);
//    }
//    public void stopTurn(){
//        hw.sorter.setPower(0.0);
//    }


    public void shooter(double targetVelocity) {
        hw.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.shooter.setVelocityPIDFCoefficients(0.002, 0.0001, 0.0001, 12.0);
        hw.shooter.setVelocity(targetVelocity);

    }
    public void shooterstop() {
        hw.shooter.setVelocity(0);

    }
    public void pull() {
        hw.pusher.setPosition(0.0);

    }
}
