package org.firstinspires.ftc.teamcode.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.TurretPID;

/**
 * Dashboard tuner for TurretPID.
 *
 * Run this OpMode, then open FTC Dashboard -> Config -> TurretPIDTuner.
 * Tune KP/KD first (FF off), then enable FF and tune KS/KV.
 */
@Config
@TeleOp(name = "Turret PIDF Tuner", group = "Tuning")
public class TurretPIDTuner extends LinearOpMode {

    // -------- Dashboard tunables --------
    public static String TURRET_MOTOR_NAME = "turret";

    // Step target (ticks)
    public static double TARGET_TICKS = 0;

    // Auto toggle between 0 and TARGET_TICKS
    public static boolean AUTO_TOGGLE = false;
    public static double TOGGLE_PERIOD_SEC = 2.0;

    // PID gains
    public static double KP = 0.012;
    public static double KI = 0.0;
    public static double KD = 0.0008;

    // Feedforward
    public static boolean ENABLE_FF = true;
    public static double KS = 0.06;
    public static double KV = 0.0009;

    // Output clamp
    public static double MAX_OUTPUT = 1.0;

    // Estimate targetVel from target changes (recommended)
    public static boolean USE_TARGET_VEL = true;
    // ----------------------------------

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, TURRET_MOTOR_NAME);

        // Custom power control
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TurretPID pid = new TurretPID();
        pid.reset();

        double lastTarget = TARGET_TICKS;
        long lastLoopNanos = 0;

        boolean toggleState = false;
        double lastToggleTime = 0.0;

        telemetry.addLine("Turret PIDF Tuner ready.");
        telemetry.addLine("Dashboard -> Config -> TurretPIDTuner");
        telemetry.update();

        waitForStart();
        pid.reset();

        while (opModeIsActive()) {
            long now = System.nanoTime();
            if (lastLoopNanos == 0) lastLoopNanos = now;
            double dt = (now - lastLoopNanos) / 1e9;
            lastLoopNanos = now;
            if (dt <= 0 || dt > 0.1) dt = 0.02;

            // Toggle target if enabled
            double elapsed = getRuntime();
            if (AUTO_TOGGLE) {
                if ((elapsed - lastToggleTime) >= TOGGLE_PERIOD_SEC) {
                    toggleState = !toggleState;
                    lastToggleTime = elapsed;
                }
            }

            double commandedTarget = AUTO_TOGGLE ? (toggleState ? TARGET_TICKS : 0.0) : TARGET_TICKS;

            // Compute target velocity (ticks/sec) from target change
            double targetVel = 0.0;
            if (USE_TARGET_VEL) {
                targetVel = (commandedTarget - lastTarget) / dt;
            }
            lastTarget = commandedTarget;

            // Apply coefficients from Dashboard
            double ks = ENABLE_FF ? KS : 0.0;
            double kv = ENABLE_FF ? KV : 0.0;
            pid.setCoefficients(KP, KI, KD, ks, kv, MAX_OUTPUT);



            // Run controller
            pid.setTarget(commandedTarget, targetVel);
            double current = turret.getCurrentPosition();
            double power = pid.update(current);
            turret.setPower(power);

            // Telemetry (Dashboard + DS)
            telemetry.addData("targetTicks", commandedTarget);
            telemetry.addData("currentTicks", current);
            telemetry.addData("errorTicks", commandedTarget - current);
            telemetry.addData("targetVel(t/s)", targetVel);
            telemetry.addData("power", power);
            telemetry.addData("AUTO_TOGGLE", AUTO_TOGGLE);
            telemetry.addData("ENABLE_FF", ENABLE_FF);
            telemetry.update();
        }

        turret.setPower(0.0);
    }
}