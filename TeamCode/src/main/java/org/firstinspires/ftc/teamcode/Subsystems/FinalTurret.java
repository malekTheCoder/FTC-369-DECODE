package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FinalTurret {

    public enum Mode {
        ODOMETRY_AUTO_MODE,
        LIMELIGHT_ASSIST_MODE,
        LIMELIGHT_BASIC_MODE,
        MANUAL_RESET_MODE,
        LOCKED_TURRET_MODE
    }

    private final UpdatedTurret turret;
    private Limelight3A limelight;
    private LLResult llResult;

    private Mode turretMode = Mode.ODOMETRY_AUTO_MODE;


    private double botErrorDeg = 0.0;
    private boolean llHasTarget = false;
    private double llTxDeg = 0.0;

    // Limelight last-seen memory + grace period
    private double llLastTxDeg = 0.0;
    private long llLastSeenNanos = 0;
    private double llGracePeriodSec = 0.25;
    private double llFallbackToOdoSec = 0.8;

    // Soft limits for turret ticks (note: your ticks are negative)
    // Rename for clarity: minAllowedTicks is the MORE negative bound (e.g. -833)
    // maxAllowedTicks is the LESS negative bound (e.g. -20)
    private int minAllowedTicks = -833;
    private int maxAllowedTicks = -20;

    // Manual reset inputs
    private boolean manualResetHeld = false;
    private double manualResetPower = 0.7; // magnitude, sign set by direction
    private int manualResetDirection = -1;
    private boolean wasManualResetHeld = false;
    private boolean aimBasic = false;

    private double manualControl;



    // limelight pid

    private double ll_kP = 0.035;
    private double ll_kD = 0.0001;
    private double ll_kS = 0.00;

    private double ll_maxOutput = 0.8;
    private double ll_deadbandDeg = 0;

    private double ll_prevErr = 0.0;
    private long ll_prevTimeNanos = 0;

    public FinalTurret(HardwareMap hardwareMap) {
        turret = new UpdatedTurret(hardwareMap.get(DcMotorEx.class, "turret"));
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0);

    }




    public void setTurretMode(Mode turretMode) {
        // If we enter LOCKED, stop any accumulated LL integral
        if (turretMode == Mode.LOCKED_TURRET_MODE) {
            ll_prevErr = 0.0;
            llLastTxDeg = 0.0;
            llLastSeenNanos = 0;
        }
        this.turretMode = turretMode;
    }

    public void setAimBasic(boolean aimBasic) {
        this.aimBasic = aimBasic;
    }

    public void setManualControl(double manualControl){
        this.manualControl = manualControl;

        if (Math.abs(manualControl) < 0.05){
            manualControl = 0;
        }
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

                // turn motor in the direction to go to the start position
                if (manualResetHeld) {
                    wasManualResetHeld = true;
                    turret.manual(manualResetPower * manualResetDirection);

                } else {
                    // once released reset the encoder
                    if (wasManualResetHeld) {
                        turret.manual(0.0);
                        turret.resetPosition();

                        wasManualResetHeld = false;

                        // switch to odo
                        setTurretMode(Mode.ODOMETRY_AUTO_MODE);
                    } else {

                        turret.manual(0.0);
                    }
                }

                break;
            }

            case LIMELIGHT_ASSIST_MODE: {

                llResult = limelight.getLatestResult();

                long now = System.nanoTime();

                boolean hasTargetNow = false;
                double txNow = 0.0;

                if (llResult != null) {
                    if (llResult.isValid()) {
                        hasTargetNow = true;
                        txNow = llResult.getTx();

                        // Save last good detection
                        llLastTxDeg = txNow;
                        llLastSeenNanos = now;
                    }
                }


                boolean useLast = false;
                double ageSec = 999.0;

                if (llLastSeenNanos != 0) {
                    ageSec = (now - llLastSeenNanos) / 1e9;
                    if (ageSec <= llGracePeriodSec) {
                        useLast = true;
                    }
                }

                boolean fallbackToOdo = false;

                // If we haven't seen a valid target for too long, use odometry aiming instead
                if (llLastSeenNanos == 0) {
                    fallbackToOdo = true;
                } else {
                    if (ageSec > llFallbackToOdoSec) {
                        fallbackToOdo = true;
                    }
                }

                // This is the tx we will aim with
                double txToUse = 0.0;
                if (useLast) {
                    txToUse = llLastTxDeg;
                }

                // ----- Compute error (aim using last detection if within grace) -----
                double error = 0.0;
                if (useLast) {
                    error = -txToUse;
                }

                // ----- PD terms -----
                double derivative = error - ll_prevErr;
                ll_prevErr = error;

                double pTerm = ll_kP * error;
                double dTerm = ll_kD * derivative;

                double output = pTerm + dTerm;

                // ----- Deadband + kS -----
                if (Math.abs(error) <= ll_deadbandDeg) {
                    output = 0.0;
                } else {
                    if (error > 0) {
                        output += Math.abs(ll_kS);
                    }
                    if (error < 0) {
                        output -= Math.abs(ll_kS);
                    }
                }

                // ----- Clamp output -----
                if (output > ll_maxOutput) {
                    output = ll_maxOutput;
                }
                if (output < -ll_maxOutput) {
                    output = -ll_maxOutput;
                }

                // ----- Soft limits (NEGATIVE power drives ticks more negative) -----
                double curTicks = turret.getCurrentTicks();

                if (curTicks <= minAllowedTicks) {
                    if (output < 0) {
                        output = 0.0;
                    }
                }

                if (curTicks >= maxAllowedTicks) {
                    if (output > 0) {
                        output = 0.0;
                    }
                }

                if (fallbackToOdo) {
                    // Fall back to odometry aiming (stay in LIMELIGHT_ASSIST_MODE)
                    turret.update(botErrorDeg);
                    turret.aimPIDF();

                    // Reset LL PD memory so it doesn't spike when we reacquire
                    ll_prevErr = 0.0;

                } else {

                    if (!useLast) {
                        // No recent detection (but not old enough to fallback): stop and reset D memory
                        output = 0.0;
                        ll_prevErr = 0.0;

                        derivative = 0.0;
                        pTerm = 0.0;
                        dTerm = 0.0;
                        error = 0.0;
                    }

                    // Apply LL output (raw)
                    turret.setPowerRaw(output);
                }

                break;
            }

            case LIMELIGHT_BASIC_MODE: {

                if (Math.abs(manualControl) > 0) {
                    aimBasic = false;
                    turret.manual(manualControl);

                    // reset LL D memory while manually driving
                    ll_prevErr = 0.0;
                    ll_prevTimeNanos = 0;

                } else {
                    aimBasic = true;
                }

                if (aimBasic) {

                    llResult = limelight.getLatestResult();

                    boolean hasTargetNow = false;
                    double txNow = 0.0;

                    if ((llResult != null) && (llResult.isValid())) {
                            hasTargetNow = true;
                            txNow = llResult.getTx();
                    }

                    if (hasTargetNow) {

                        double error = -txNow;

                        long now = System.nanoTime();
                        double derivative = 0.0;

                        if (ll_prevTimeNanos != 0) {
                            double dt = (now - ll_prevTimeNanos) / 1e9;
                            if (dt > 1e-6) {
                                derivative = (error - ll_prevErr) / dt;
                            }
                        }

                        ll_prevErr = error;
                        ll_prevTimeNanos = now;

                        double output = (ll_kP * error) + (ll_kD * derivative);

                        // deadband and ks
                        if (Math.abs(error) <= ll_deadbandDeg) {
                            output = 0.0;
                        } else {
                            if (error > 0) {
                                output += Math.abs(ll_kS);
                            }
                            if (error < 0) {
                                output -= Math.abs(ll_kS);
                            }
                        }

                        // Clamp output
                        if (output > ll_maxOutput) {
                            output = ll_maxOutput;
                        }
                        if (output < -ll_maxOutput) {
                            output = -ll_maxOutput;
                        }

                        double curTicks = turret.getCurrentTicks();

                        if (curTicks <= minAllowedTicks) {
                            if (output < 0) {
                                output = 0.0;
                            }
                        }

                        if (curTicks >= maxAllowedTicks) {
                            if (output > 0) {
                                output = 0.0;
                            }
                        }

                        turret.setPowerRaw(output);

                    } else {
                        turret.setPowerRaw(0.0);
                        ll_prevErr = 0.0;
                        ll_prevTimeNanos = 0;
                    }

                }

                if (!aimBasic) {
                    ll_prevErr = 0.0;
                    ll_prevTimeNanos = 0;
                }

                break;
            }

            case ODOMETRY_AUTO_MODE: {
                turret.update(botErrorDeg);
                turret.aimPIDF();
                break;
            }

            case LOCKED_TURRET_MODE: {
                turret.setPowerRaw(0.0);
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
        // Backwards-compatible overload
        setLimelightPid(kP, kD, this.ll_kS, maxOutput, deadbandDeg);
    }

    public void setLimelightPid(double kP, double kD, double kS, double maxOutput, double deadbandDeg) {
        this.ll_kP = kP;
        this.ll_kD = kD;
        this.ll_kS = Math.abs(kS);
        this.ll_maxOutput = Math.abs(maxOutput);
        this.ll_deadbandDeg = Math.abs(deadbandDeg);
    }

    public UpdatedTurret getTurretLogic() {
        return turret;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("MO0DE: ", turretMode.toString());
        telemetry.addData("TX:", llTxDeg);
        turret.addTelemetry(telemetry);
    }
}