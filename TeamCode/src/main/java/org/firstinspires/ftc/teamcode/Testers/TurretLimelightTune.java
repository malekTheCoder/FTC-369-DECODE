package org.firstinspires.ftc.teamcode.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Disabled
@Config
@TeleOp(name = "Turret Limelight tuning", group = "TEST")
public class TurretLimelightTune extends OpMode {

    // -------- Dashboard tunables --------
    public static double kP = 0.01;
    public static double kD = 0.0;   // start at 0, then tune
    public static double kS = 0.07;

    public static double maxOutput = 0.45;
    public static double deadbandDeg = 0.4;

    // Turret limits (ticks are negative)
    public static int minAllowedTicks = -833; // MORE negative bound
    public static int maxAllowedTicks = -20;  // LESS negative bound
    // -----------------------------------

    private DcMotorEx turret;
    private Limelight3A limelight;

    private double prevErr = 0.0;
    private long prevTimeNanos = 0;

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

        LLResult res = limelight.getLatestResult();

        boolean hasTarget = false;
        double tx = 0.0;

        if (res != null) {
            if (res.isValid()) {
                hasTarget = true;
                tx = res.getTx();
            }
        }

        double error = 0.0;
        double derivative = 0.0;
        double dt = 0.0;

        if (hasTarget) {
            error = -tx;

            long now = System.nanoTime();

            if (prevTimeNanos != 0) {
                dt = (now - prevTimeNanos) / 1e9;
                if (dt > 1e-6) {
                    derivative = (error - prevErr) / dt;
                }
            }

            prevErr = error;
            prevTimeNanos = now;

        } else {
            prevErr = 0.0;
            prevTimeNanos = 0;
        }

        double out = 0.0;

        if (hasTarget) {
            out = (kP * error) + (kD * derivative);

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

            if (out > maxOutput) {
                out = maxOutput;
            }
            if (out < -maxOutput) {
                out = -maxOutput;
            }

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

        } else {
            out = 0.0;
        }

        turret.setPower(out);

        // Graph these in Dashboard
        telemetry.addData("has", hasTarget ? 1 : 0);
        telemetry.addData("tx", tx);
        telemetry.addData("err", error);
        telemetry.addData("dt", dt);
        telemetry.addData("derr", derivative);
        telemetry.addData("out", out);
        telemetry.addData("ticks", turret.getCurrentPosition());
        telemetry.update();
    }
}