package org.firstinspires.ftc.teamcode.Subsystems;


public class TurretPID {

    public double kP = 0.011;
    public double kI = 0.0; // wont use
    public double kD = 0.0004;

    // public double kS = 0.09; // static friction feedforward
    public double kS = 0.06; // static friction feedforward




    //    public double kV = 0.0015; // velocity feedforward - need to tune
    public double kV = 0.0012; // velocity feedforward - need to tune

    public double maxOutput = 1.0;

    // wont use integral for now
    public double integralMax = 2000.0;


    private double target = 0.0;
    private double targetVel = 0.0; // ticks/second

    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastTimeNanos = 0;


    public void setTarget(double target) {
        this.target = target;
        this.targetVel = 0.0;
    }

    public void setTarget(double target, double targetVel) {
        this.target = target;
        this.targetVel = targetVel;
    }

    public void setCoefficients(double kP, double kI, double kD,
                                double kS, double kV,
                                double maxOutput) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
        this.maxOutput = maxOutput;
    }


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

        // Feedforward: kS helps overcome static friction
        double ff = 0.0;

       // If target is still changing, use target velocity direction
        if (Math.abs(targetVel) > 1e-6) {
            ff += kS * Math.signum(targetVel);
        }
       // Otherwise, if we are still not at the target, push in error direction
        else if (Math.abs(error) > 1.0) {  // 1 tick deadband
            ff += kS * Math.signum(error);
        }

       // Velocity feedforward
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
