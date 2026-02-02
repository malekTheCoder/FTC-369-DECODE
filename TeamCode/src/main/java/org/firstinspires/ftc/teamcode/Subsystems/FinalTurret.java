package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FinalTurret {

    public enum Mode {
        ODOMETRY_AUTO_MODE,
        LIMELIGHT_ASSIST_MODE,
        MANUAL_RESET_MODE,
        LOCKED_TURRET_MODE
    }

    private final UpdatedTurret turret;
    private Mode turretMode = Mode.ODOMETRY_AUTO_MODE;


    private double botErrorDeg = 0.0;
    private boolean llHasTarget = false;
    private double llTxDeg = 0.0;

    // Manual reset inputs
    private boolean manualResetHeld = false;
    private double manualResetPower = 0.7; // magnitude, sign set by direction
    private int manualResetDirection = -1;
    private boolean wasManualResetHeld = false;



    // limelight pid

    // Tune these on the real bot. Start conservative.
    private double ll_kP = 0.03;
    private double ll_kD = 0.001;

    private double ll_maxOutput = 0.45;
    private double ll_deadbandDeg = 0.6;

    private double ll_prevErr = 0.0;

    public FinalTurret(HardwareMap hardwareMap) {
        turret = new UpdatedTurret(hardwareMap.get(DcMotorEx.class, "turret"));
    }

    // ---------------- Mode control ----------------

    public void setTurretMode(Mode turretMode) {
        // If we enter LOCKED, stop any accumulated LL integral
        if (turretMode == Mode.LOCKED_TURRET_MODE) {
            ll_prevErr = 0.0;
        }
        this.turretMode = turretMode;
    }

    public Mode getTurretMode() {
        return turretMode;
    }


    // ODO auto
    public void setBotErrorDeg(double botErrorDeg) {
        this.botErrorDeg = botErrorDeg;
    }

    // Limelight
    public void setLimelightHasTarget(boolean hasTarget) {
        this.llHasTarget = hasTarget;
    }

    public void setLimelightTxDeg(double txDeg) {
        this.llTxDeg = txDeg;
    }

    // Manual reset
    public void setManualResetHeld(boolean held) {
        this.manualResetHeld = held;
    }


    /** Power magnitude used during MANUAL_RESET while held (0..1). */
    public void setManualResetPower(double power) {
        this.manualResetPower = clamp(power, 0.0, 1.0);
    }

    public void resetPosition() {
        turret.resetPosition();
    }


    public void update() {
        switch (turretMode) {

            case MANUAL_RESET_MODE: {

                // While held: drive turret in chosen direction.
                if (manualResetHeld) {
                    wasManualResetHeld = true;
                    turret.manual(manualResetPower * manualResetDirection);
                } else {
                    // On release: stop and reset encoder FIRST, then switch back to ODO tracking.
                    if (wasManualResetHeld) {
                        turret.manual(0.0);
                        turret.resetPosition();

                        wasManualResetHeld = false;

                        // Switch AFTER reset so ODO tracking starts next loop.
                        setTurretMode(Mode.ODOMETRY_AUTO_MODE);
                    } else {
                        // Not held and no recent release: keep turret stopped.
                        turret.manual(0.0);
                    }
                }

                break;
            }

            case LIMELIGHT_ASSIST_MODE: {

                // If LL sees a target: center turret using LL tx PID.
                // If not: fall back to odometry aiming, but stay in LIMELIGHT_ASSIST_MODE.
                if (llHasTarget) {
                    double err = llTxDeg;

                    // Deadband (don’t chase tiny noise)
                    if (Math.abs(err) <= ll_deadbandDeg) {
                        err = 0.0;
                    }

                    // Simple PID (dt-less, assumes ~constant loop time)
                    double d = err - ll_prevErr;
                    ll_prevErr = err;

                    double out = (ll_kP * err) + (ll_kD * d);
                    out = clamp(out, -ll_maxOutput, ll_maxOutput);

                    turret.manual(out);

                } else {
                    // No target: use odometry aim as fallback.
                    turret.update(botErrorDeg);
                    turret.aimPIDF();

                    // Reset LL PID memory so it doesn't jump when the tag reappears.
                    ll_prevErr = 0.0;
                }

                break;
            }

            case ODOMETRY_AUTO_MODE: {
                turret.update(botErrorDeg);
                turret.aimPIDF();
                break;
            }

            case LOCKED_TURRET_MODE: {
                turret.manual(0);
                break;
            }
        }
    }


    public boolean isOnTargetDeg(double degTolerance) {
        return turret.isOnTarget(degTolerance);
    }

    public double getCurrentTicks() {
        return turret.getCurrentTicks();
    }

    public double getTargetTicks() {
        return turret.getTargetTicks();
    }

    public double getTargetVelocityTicksPerSec() {
        return turret.getTargetVelocityTicksPerSec();
    }

    public TurretPID getPid() {
        return turret.getPid();
    }

    public void setPidCoefficients(double kP, double kI, double kD, double kS, double kV, double maxOutput) {
        turret.getPid().setCoefficients(kP, kI, kD, kS, kV, maxOutput);
    }

    public void setLimelightPid(double kP, double kD, double maxOutput, double deadbandDeg) {
        this.ll_kP = kP;
        this.ll_kD = kD;
        this.ll_maxOutput = Math.abs(maxOutput);
        this.ll_deadbandDeg = Math.abs(deadbandDeg);
    }

    public UpdatedTurret getTurretLogic() {
        return turret;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}