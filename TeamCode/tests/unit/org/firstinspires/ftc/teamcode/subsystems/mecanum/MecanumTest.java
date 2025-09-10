package org.firstinspires.ftc.teamcode.subsystems.mecanum;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.ArgumentCaptor;
import org.mockito.Captor;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumConstants.MAX_ANGULAR_VEL;
import static org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumConstants.POWER_SCALE_FACTOR;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
class MecanumTest {

    private Hardware hw;

    private DcMotorEx lf;
    private DcMotorEx lb;
    private DcMotorEx rf;
    private DcMotorEx rb;

    private MecanumSubsystem mech;

    @Captor ArgumentCaptor<Double> powerCaptor;

    @BeforeEach
    void setup() {
        // Create motor mocks
        lf = mock(DcMotorEx.class);
        lb = mock(DcMotorEx.class);
        rf = mock(DcMotorEx.class);
        rb = mock(DcMotorEx.class);

        // Default stubbing used by some methods in your class
        when(rf.getCurrentPosition()).thenReturn(0);
        when(rb.getCurrentPosition()).thenReturn(0);
        when(lf.getCurrentPosition()).thenReturn(0);
        when(lb.getCurrentPosition()).thenReturn(0);

        // Create a mock Hardware container and assign motor fields
        // (Direct field assignment on a Mockito mock works for public fields.)
        hw = mock(Hardware.class, withSettings().defaultAnswer(RETURNS_DEEP_STUBS));
//        hw.lf = lf;
//        hw.lb = lb;
//        hw.rf = rf;
//        hw.rb = rb;

        // Instantiate the subsystem (constructor will set directions/modes/zero-power/power)
        mech = new MecanumSubsystem(hw);
    }

    // --- Helper for approximate equality ---
    private static void assertClose(double expected, double actual, double tol) {
        assertTrue(Math.abs(expected - actual) <= tol,
                () -> "Expected ≈ " + expected + " but was " + actual);
    }

    @Test
    void fieldOrientedMove_strafeX_only_scalesCorrectly() {
        // x = 1, y = 0, z = 0, theta = 0
        mech.fieldOrientedMove(1.0, 0.0, 0.0, 0.0);

        // With theta=0, the raw wheel values should all be +1 before normalization,
        // then multiplied by POWER_SCALE_FACTOR.
        verify(rf).setPower(POWER_SCALE_FACTOR);
        verify(lf).setPower(POWER_SCALE_FACTOR);
        verify(rb).setPower(POWER_SCALE_FACTOR);
        verify(lb).setPower(POWER_SCALE_FACTOR);
    }

    @Test
    void fieldOrientedMove_normalizes_whenOver1() {
        // Choose values likely to exceed magnitude 1 before normalization
        double x = 1.0, y = 1.0, z = 1.0, theta = 0.0;
        mech.fieldOrientedMove(x, y, z, theta);

        // Capture powers
        ArgumentCaptor<Double> rfP = ArgumentCaptor.forClass(Double.class);
        ArgumentCaptor<Double> lfP = ArgumentCaptor.forClass(Double.class);
        ArgumentCaptor<Double> rbP = ArgumentCaptor.forClass(Double.class);
        ArgumentCaptor<Double> lbP = ArgumentCaptor.forClass(Double.class);

        verify(rf).setPower(rfP.capture());
        verify(lf).setPower(lfP.capture());
        verify(rb).setPower(rbP.capture());
        verify(lb).setPower(lbP.capture());

        double prf = rfP.getValue();
        double plf = lfP.getValue();
        double prb = rbP.getValue();
        double plb = lbP.getValue();

        // All magnitudes must be <= POWER_SCALE_FACTOR after normalization
        assertTrue(Math.abs(prf) <= POWER_SCALE_FACTOR + 1e-9);
        assertTrue(Math.abs(plf) <= POWER_SCALE_FACTOR + 1e-9);
        assertTrue(Math.abs(prb) <= POWER_SCALE_FACTOR + 1e-9);
        assertTrue(Math.abs(plb) <= POWER_SCALE_FACTOR + 1e-9);

        // Also verify the exact expected values from the raw -> normalized -> scaled pipeline
        // Raw (theta=0):
        // rf = -newY + newX - z = -(y) + x - z = -1 + 1 - 1 = -1
        // lf =  newY + newX + z =   1 + 1 + 1 =  3
        // rb =  newY + newX - z =   1 + 1 - 1 =  1
        // lb = -newY + newX + z =  -1 + 1 + 1 =  1
        double rfRaw = -1, lfRaw = 3, rbRaw = 1, lbRaw = 1;
        double largest = Math.max(Math.abs(rfRaw),
                Math.max(Math.abs(lfRaw), Math.max(Math.abs(rbRaw), Math.abs(lbRaw))));
        double expectRf = rfRaw / largest * POWER_SCALE_FACTOR;
        double expectLf = lfRaw / largest * POWER_SCALE_FACTOR;
        double expectRb = rbRaw / largest * POWER_SCALE_FACTOR;
        double expectLb = lbRaw / largest * POWER_SCALE_FACTOR;

        double tol = 1e-9;
        assertClose(expectRf, prf, tol);
        assertClose(expectLf, plf, tol);
        assertClose(expectRb, prb, tol);
        assertClose(expectLb, plb, tol);
    }

    @Test
    void motorProcess_clampsToMaxAngularVel_andPreservesRatios() {
        // Drive a command through the "main" layer using partialMove to create non-trivial ratios.
        // Values chosen to likely exceed MAX_ANGULAR_VEL so clamping branch is exercised.
        mech.partialMove(true,
                2.0,   // verticalVel
                1.0,   // horizontalVel
                1.5);  // rotationalVel

        mech.motorProcess(); // should issue setVelocity(...) on each motor (radians/sec)

        // Capture setVelocity calls
        ArgumentCaptor<Double> rfVel = ArgumentCaptor.forClass(Double.class);
        ArgumentCaptor<Double> rbVel = ArgumentCaptor.forClass(Double.class);
        ArgumentCaptor<Double> lfVel = ArgumentCaptor.forClass(Double.class);
        ArgumentCaptor<Double> lbVel = ArgumentCaptor.forClass(Double.class);

        verify(rf).setVelocity(rfVel.capture(), eq(AngleUnit.RADIANS));
        verify(rb).setVelocity(rbVel.capture(), eq(AngleUnit.RADIANS));
        verify(lf).setVelocity(lfVel.capture(), eq(AngleUnit.RADIANS));
        verify(lb).setVelocity(lbVel.capture(), eq(AngleUnit.RADIANS));

        double vRF = rfVel.getValue();
        double vRB = rbVel.getValue();
        double vLF = lfVel.getValue();
        double vLB = lbVel.getValue();

        // Confirm clamp
        assertTrue(Math.abs(vRF) <= MAX_ANGULAR_VEL + 1e-9);
        assertTrue(Math.abs(vRB) <= MAX_ANGULAR_VEL + 1e-9);
        assertTrue(Math.abs(vLF) <= MAX_ANGULAR_VEL + 1e-9);
        assertTrue(Math.abs(vLB) <= MAX_ANGULAR_VEL + 1e-9);

        // Ratios should be preserved (within tolerance) after uniform scaling
        // Compare pairwise ratios where denominators are not ~0
        double tol = 1e-6;

        // Compute expected raw (what partialMove computes) to compare ratio after scaling:
        // Using the same formula as partialMove (45° trig factors simplify with *sqrt(2))
        // We’ll rebuild the raw target wheel velocities from inputs to check that
        // v_out = raw * k where |k| <= 1 to clamp to MAX_ANGULAR_VEL.
        final double c = Math.cos(Math.toRadians(45));
        final double s = Math.sin(Math.toRadians(45));
        final double sqrt2 = 1.41421356237;

        double vertical = 2.0, horizontal = 1.0, rotational = 1.5;
        double rbRaw = (vertical * c + horizontal * s + rotational * s) * sqrt2;
        double rfRaw = (-horizontal * c + vertical * s + rotational * s) * sqrt2;
        double lfRaw = (vertical * c + horizontal * s - rotational * s) * sqrt2;
        double lbRaw = (-horizontal * c + vertical * s - rotational * s) * sqrt2;

        // k = v_out / raw should be the same for all (uniform scaling), ignoring sign/zero edge cases
        double kRF = vRF / rfRaw;
        double kRB = vRB / rbRaw;
        double kLF = vLF / lfRaw;
        double kLB = vLB / lbRaw;

        // If any raw is ~0, skip that comparison to avoid numeric noise
        if (Math.abs(rfRaw) > 1e-9 && Math.abs(rbRaw) > 1e-9) assertClose(kRF, kRB, tol);
        if (Math.abs(rfRaw) > 1e-9 && Math.abs(lfRaw) > 1e-9) assertClose(kRF, kLF, tol);
        if (Math.abs(rfRaw) > 1e-9 && Math.abs(lbRaw) > 1e-9) assertClose(kRF, kLB, tol);
    }

    @Test
    void normalizedFunction_isOdd_bounded_andZeroAtZero() {
        // Zero at origin
        assertEquals(0.0, MecanumSubsystem.normalizedFunction(0.0), 1e-12);

        // Odd symmetry
        double a = 0.37;
        double fa = MecanumSubsystem.normalizedFunction(a);
        double fna = MecanumSubsystem.normalizedFunction(-a);
        assertClose(fa, -fna, 1e-12);

        // Bounded in [-1, 1] for |t|<=1
        double[] samples = {-1.0, -0.8, -0.2, 0.2, 0.8, 1.0};
        for (double t : samples) {
            double f = MecanumSubsystem.normalizedFunction(t);
            assertTrue(f >= -1.0000001 && f <= 1.0000001,
                    "normalizedFunction should be bounded in [-1,1], got " + f + " for t=" + t);
        }

        // Monotone increasing on [0,1] (sampled)
        double prev = MecanumSubsystem.normalizedFunction(0.0);
        for (double t = 0.1; t <= 1.0; t += 0.1) {
            double cur = MecanumSubsystem.normalizedFunction(t);
            assertTrue(cur >= prev - 1e-12,
                    "normalizedFunction should be non-decreasing on [0,1]");
            prev = cur;
        }
    }

    @Test
    void move_setsRawPowers_robotCentric() {
        mech.move(true, /*vertical*/ 0.5, /*horizontal*/ 0.0, /*rotational*/ 0.0);

        // With horizontal=0, rotational=0, vertical>0:
        // rf =  v*cos45 = 0.5*sqrt(2)/2 * sqrt(2) = 0.5
        // lf =  v*cos45 = 0.5
        // rb =  v*cos45 = 0.5
        // lb =  v*cos45 = 0.5
        verify(rf).setPower(0.5);
        verify(lf).setPower(0.5);
        verify(rb).setPower(0.5);
        verify(lb).setPower(0.5);
    }

    @Test
    void turnOffInternalPID_switchesModes() {
        mech.turnOffInternalPID();

        verify(rf, atLeastOnce()).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verify(lf, atLeastOnce()).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verify(rb, atLeastOnce()).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verify(lb, atLeastOnce()).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
