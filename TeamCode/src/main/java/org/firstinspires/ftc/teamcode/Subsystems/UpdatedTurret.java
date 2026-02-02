package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.TurretPID;

public class UpdatedTurret {
    private double totalTurnTicks = 853.0;
    private double totalDegrees = 320.0;

    private double maxSafeTicks = 840.0;
    private double minSafeTicks = 15.0;
    private double ticksPerDegree = totalTurnTicks / totalDegrees;


    private double currentPositionTicks;
    private double currentPositionDegrees;
    private double targetPositionTurretTicks;
    private double targetPositionTurretDegrees;
    private double minDegrees = 20;
    private double maxDegrees = 340;

    private double safeMinDegrees = 25;
    private double safeMaxDegrees = 335;

    private boolean inDeadZone;

    private DcMotorEx turret;

    // Reusable PIDF controller
    private final TurretPID pid = new TurretPID();


    private double targetVelTicksPerSec = 0.0;
    private double lastTargetTicks = 0.0;
    private long lastTargetTimeNanos = 0;


    private double filteredTargetTicks = 0.0;
    private boolean filterInitialized = false;

    // 0..1 (higher = faster, lower = smoother). Start at 0.20.
    private static final double TARGET_ALPHA = 0.125;

    public UpdatedTurret(DcMotorEx turret) {
        this.turret = turret;
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        inDeadZone = true;
    }

    public void manual(double stickX){
        // Still RUN_WITHOUT_ENCODER; direct manual power
        turret.setPower(-stickX * 0.3);
    }

    public void resetPosition(){
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pid.reset();
        lastTargetTimeNanos = 0;
        lastTargetTicks = 0.0;
        targetVelTicksPerSec = 0.0;
        filterInitialized = false;
        filteredTargetTicks = 0.0;
    }

    public void update (double botErrorDeg){
        currentPositionTicks = turret.getCurrentPosition();
        currentPositionDegrees = normalize360(minDegrees + (-currentPositionTicks / ticksPerDegree));

        // targetPositionTurretDegrees = normalize360(360 - botErrorDeg);
        targetPositionTurretDegrees = clamp(normalize360(360 - botErrorDeg), safeMinDegrees, safeMaxDegrees);
        targetPositionTurretTicks = -(targetPositionTurretDegrees - minDegrees) * ticksPerDegree;

        // Low-pass filter the target ticks so tiny botErrorDeg noise doesn't cause jitter
        if (!filterInitialized) {
            filteredTargetTicks = targetPositionTurretTicks;
            filterInitialized = true;
        } else {
            filteredTargetTicks = filteredTargetTicks + TARGET_ALPHA * (targetPositionTurretTicks - filteredTargetTicks);
        }

        updateTargetVelocity();


//        t.addData("bot error deg", botErrorDeg);
//        t.addData("curr ticks", currentPositionTicks);
//        t.addData("curr deg", currentPositionDegrees);
//        t.addLine("---------------");
//        t.addData("target ticks", targetPositionTurretTicks);
//        t.addData("target deg", targetPositionTurretDegrees);
//        t.addData("target vel (ticks/sec)", targetVelTicksPerSec);
//        t.addLine("---------------");
//        t.addData("in deadzone", inDeadZone);
//        t.addData("on target", isOnTarget(5));
//        t.addLine("---------------");
//        t.update();

    }



    private static double clamp(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    private static double normalize360(double deg) {
        deg = deg % 360;
        if (deg < 0) deg += 360;
        return deg;
    }



    public void aimPIDF(){
        pid.setTarget(filteredTargetTicks, targetVelTicksPerSec);
        double currentTicks = turret.getCurrentPosition();
        double power = pid.update(currentTicks);
        turret.setPower(power);
    }

    public TurretPID getPid(){
        return pid;
    }

    public double getTargetVelocityTicksPerSec(){
        return targetVelTicksPerSec;
    }


    private void updateTargetVelocity() {
        long now = System.nanoTime();

        if (lastTargetTimeNanos == 0) {
            lastTargetTimeNanos = now;
            lastTargetTicks = filteredTargetTicks;
            targetVelTicksPerSec = 0.0;
            return;
        }

        double dt = (now - lastTargetTimeNanos) / 1e9;
        lastTargetTimeNanos = now;

        // error time correction
        if (dt <= 0 || dt > 0.1) dt = 0.02;

        targetVelTicksPerSec = (filteredTargetTicks - lastTargetTicks) / dt;
        lastTargetTicks = filteredTargetTicks;
    }

    public boolean isOnTarget(double degTolerance){
        double tickDiff = Math.abs(turret.getCurrentPosition() - targetPositionTurretTicks);
        return (tickDiff/ticksPerDegree) < degTolerance;
    }

    public double getTargetTicks() {
        return filteredTargetTicks;
    }

    public double getCurrentTicks() {
        return turret.getCurrentPosition();
    }



}
