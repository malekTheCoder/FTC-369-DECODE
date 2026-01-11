package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.TurretPID;

/**
 * FTC Dashboard tuner for the turret PIDF.
 *
 * How to use:
 * 1) Set the motor name in the RC config (default: "turret").
 * 2) Run this TeleOp.
 * 3) Use FTC Dashboard to change TARGET_TICKS, gains, and limits live.
 * 4) Watch target/current/error/power to tune.
 */
@Config
@TeleOp(name = "Turret PIDF Tuner", group = "Tuning")
public class TurretTuner extends LinearOpMode {



    // Step target (ticks). Change live on Dashboard.
    public static double TARGET_TICKS = 0;

    // If true, automatically toggles the target between 0 and TARGET_TICKS every TOGGLE_PERIOD_SEC.
    public static boolean AUTO_TOGGLE = false;
    public static double TOGGLE_PERIOD_SEC = 2.0;

    // Output clamps
    public static double MAX_OUTPUT = 1.0;

    // PID gains
    public static double kP = 0.012;
    public static double kI = 0.0;
    public static double kD = 0.0008;

    // Feedforward
    public static double kS = 0.06;
    public static double kV = 0.0009;

    // If true, apply kS/kV feedforward. If false, pure PID.
    public static boolean ENABLE_FF = true;

    // If true, set targetVel based on how TARGET_TICKS changes over time (recommended).
    public static boolean USE_TARGET_VEL = true;

    // -----------------------------------

    @Override
    public void runOpMode() {
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");

        // Custom control: RUN_WITHOUT_ENCODER, we compute power
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TurretPID pid = new TurretPID();

        // Telemetry -> Driver Station + Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        double lastTarget = TARGET_TICKS;
        long lastLoopNanos = 0;

        double autoTargetA = 0;
        double autoTargetB = TARGET_TICKS;
        boolean autoState = false;
        double lastToggleSec = 0;

        telemetry.addLine("Turret PIDF Tuner ready.");
        telemetry.addLine("Use Dashboard to edit TARGET_TICKS / kP,kD,kS,kV and watch telemetry.");
        telemetry.update();

        waitForStart();
        pid.reset();

        while (opModeIsActive()) {
            long now = System.nanoTime();
            if (lastLoopNanos == 0) lastLoopNanos = now;
            double dt = (now - lastLoopNanos) / 1e9;
            lastLoopNanos = now;
            if (dt <= 0 || dt > 0.1) dt = 0.02;

            // Apply dashboard tunables to PID object
            pid.kP = kP;
            pid.kI = kI;
            pid.kD = kD;
            pid.kS = ENABLE_FF ? kS : 0.0;
            pid.kV = ENABLE_FF ? kV : 0.0;
            pid.maxOutput = MAX_OUTPUT;

            // Optional auto toggle between two targets
            double elapsed = getRuntime();
            if (AUTO_TOGGLE) {
                autoTargetB = TARGET_TICKS; // keep B synced with dashboard edits
                if ((elapsed - lastToggleSec) >= TOGGLE_PERIOD_SEC) {
                    autoState = !autoState;
                    lastToggleSec = elapsed;
                }
            }

            double commandedTarget = AUTO_TOGGLE ? (autoState ? autoTargetB : autoTargetA) : TARGET_TICKS;

            // Estimate target velocity for FF
            double targetVel = 0.0;
            if (USE_TARGET_VEL) {
                targetVel = (commandedTarget - lastTarget) / dt;
            }
            lastTarget = commandedTarget;

            // Set target and update
            pid.setTarget(commandedTarget, targetVel);

            double current = turret.getCurrentPosition();
            double power = pid.update(current);
            turret.setPower(power);

            double error = commandedTarget - current;

            telemetry.addData("targetTicks", commandedTarget);
            telemetry.addData("currentTicks", current);
            telemetry.addData("errorTicks", error);
            telemetry.addData("targetVel(t/s)", targetVel);
            telemetry.addData("power", power);
            telemetry.addData("AUTO_TOGGLE", AUTO_TOGGLE);
            telemetry.addData("ENABLE_FF", ENABLE_FF);
            telemetry.update();
        }

        turret.setPower(0);
    }
}
