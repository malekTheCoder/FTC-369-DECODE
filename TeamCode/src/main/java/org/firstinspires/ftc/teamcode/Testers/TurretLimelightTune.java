package org.firstinspires.ftc.teamcode.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "Turret Limelight tuning", group = "TEST")
public class TurretLimelightTune extends OpMode {

    // -------- Dashboard tunables --------
    public static double kP = 0.01;
    public static double kD = 0.001;
    public static double kS = 0.07;

    public static double maxOutput = 0.45;
    public static double deadbandDeg = 0.4;
    public static double gracePeriodSec = 0.25;

    // Turret limits (ticks are negative)
    public static int minAllowedTicks = -833; // MORE negative bound
    public static int maxAllowedTicks = -20;  // LESS negative bound
    // -----------------------------------

    private DcMotorEx turret;
    private Limelight3A limelight;

    private double prevErr = 0.0;
    private double lastTx = 0.0;
    private long lastSeenNanos = 0;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0);

        Telemetry dash = FtcDashboard.getInstance().getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dash);
    }

    @Override
    public void loop() {

        // ----- Read Limelight -----
        LLResult res = limelight.getLatestResult();

        long now = System.nanoTime();

        boolean hasTargetNow = false;
        double txNow = 0.0;

        if (res != null) {
            if (res.isValid()) {
                hasTargetNow = true;
                txNow = res.getTx();

                // Save last good detection
                lastTx = txNow;
                lastSeenNanos = now;
            }
        }

        // Decide if we can use last detection (within grace)
        boolean useLast = false;
        double ageSec = 999.0;

        if (lastSeenNanos != 0) {
            ageSec = (now - lastSeenNanos) / 1e9;
            if (ageSec <= gracePeriodSec) {
                useLast = true;
            }
        }

        // This is the tx we will aim with
        double txToUse = 0.0;
        if (useLast) {
            txToUse = lastTx;
        }

        // ----- Compute error (aim using last detection if within grace) -----
        double error = 0.0;
        if (useLast) {
            error = -txToUse;
        }

        // ----- PD terms -----
        double derivative = error - prevErr;
        prevErr = error;

        double pTerm = kP * error;
        double dTerm = kD * derivative;

        double out = pTerm + dTerm;

        // ----- Deadband + kS -----
        if (Math.abs(error) <= deadbandDeg) {
            out = 0.0;
        } else {
            if (error > 0) {
                out += Math.abs(kS);
            }
            if (error < 0) {
                out -= Math.abs(kS);
            }
        }

        // ----- Clamp output -----
        if (out > maxOutput) {
            out = maxOutput;
        }
        if (out < -maxOutput) {
            out = -maxOutput;
        }

        // ----- Soft limits (NEGATIVE power drives ticks more negative) -----
        int curTicks = turret.getCurrentPosition();

        if (curTicks <= minAllowedTicks) {
            if (out < 0) {
                out = 0.0;
            }
        }

        if (curTicks >= maxAllowedTicks) {
            if (out > 0) {
                out = 0.0;
            }
        }

        // ----- No recent detection: stop and reset D memory -----
        if (!useLast) {
            out = 0.0;
            prevErr = 0.0;

            derivative = 0.0;
            pTerm = 0.0;
            dTerm = 0.0;
            error = 0.0;
        }

        // ----- Apply power -----
        turret.setPower(out);

        // ----- Telemetry (graph these in Dashboard) -----
        telemetry.addData("hasNow", hasTargetNow ? 1 : 0);
        telemetry.addData("useLast", useLast ? 1 : 0);
        telemetry.addData("txNow", txNow);
        telemetry.addData("txLast", lastTx);
        telemetry.addData("ageSec", ageSec);
        telemetry.addData("err", error);
        telemetry.addData("derr", derivative);
        telemetry.addData("P", pTerm);
        telemetry.addData("D", dTerm);
        telemetry.addData("out", out);
        telemetry.addData("ticks", curTicks);
        telemetry.update();
    }
}