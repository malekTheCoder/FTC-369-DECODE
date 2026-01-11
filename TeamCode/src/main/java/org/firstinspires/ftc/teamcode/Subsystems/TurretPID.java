package org.firstinspires.ftc.teamcode.Subsystems;

/**
 * Small reusable PIDF helper.
 *
 * - Call setTarget(...) each loop (optionally with target velocity for FF)
 * - Call update(measurement) each loop to get output (e.g., motor power)
 *
 * Notes:
 * - For a tracking turret, you typically keep kI = 0 to avoid windup.
 * - Feedforward here is tuned for turrets: kS*sign(v) + kV*v.
 */
public class TurretPID {

    public double kP = 0.012;
    public double kI = 0.0; // wont use
    public double kD = 0.0008;

    public double kS = 0.06; // static friction feedforward
    public double kV = 0.0009; // velocity feedforward

    // Output clamp
    public double maxOutput = 1.0;

    // wont use integral for now
    public double integralMax = 2000.0;

    // --------- State ---------
    private double target = 0.0;
    private double targetVel = 0.0; // units: target units per second (e.g., ticks/sec)

    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastTimeNanos = 0;

    /** Set target position only. Feedforward uses 0 velocity. */
    public void setTarget(double target) {
        this.target = target;
        this.targetVel = 0.0;
    }

    /** Set target position and target velocity (recommended for feedforward). */
    public void setTarget(double target, double targetVel) {
        this.target = target;
        this.targetVel = targetVel;
    }

    /**
     * Update using the provided measurement and return the control output.
     * This method internally computes dt using System.nanoTime().
     */
    public double update(double measurement) {
        long now = System.nanoTime();

        if (lastTimeNanos == 0) {
            lastTimeNanos = now;
            lastError = target - measurement;
            return 0.0;
        }

        double dt = (now - lastTimeNanos) / 1e9;
        lastTimeNanos = now;

        // Guard dt to avoid crazy spikes
        if (dt <= 0 || dt > 0.1) dt = 0.02;

        double error = target - measurement;

        // PID
        integral += error * dt;
        integral = clamp(integral, -integralMax, integralMax);

        double derivative = (error - lastError) / dt;
        lastError = error;

        double pid = kP * error + kI * integral + kD * derivative;

        // Feedforward: kS*sign(v) + kV*v
        double ff = 0.0;
        if (Math.abs(targetVel) > 1e-6) {
            ff += kS * Math.signum(targetVel);
        }
        ff += kV * targetVel;

        double out = pid + ff;
        return clamp(out, -maxOutput, maxOutput);
    }

    public void reset() {
        integral = 0.0;
        lastError = 0.0;
        lastTimeNanos = 0;
    }

    public double getTarget() {
        return target;
    }

    public double getTargetVel() {
        return targetVel;
    }

    private static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }
}
