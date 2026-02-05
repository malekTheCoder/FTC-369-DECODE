package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SimpleTurret {

    private DcMotorEx turret;


    private final double totalTurnTicks = 853.0;
    private final double totalDegrees  = 320.0;
    private final double ticksPerDegree = totalTurnTicks / totalDegrees;

    private final double minDegrees = 20.0;
    private final double safeMinDegrees = 25.0;
    private final double safeMaxDegrees = 335.0;

    private double currentPositionTicks;
    private double currentPositionDegrees;
    private double targetPositionTurretTicks;
    private double targetPositionTurretDegrees;


    // Safety limits
    private final int minSafeTicks = 15;
    private final int maxSafeTicks = 840;

    public SimpleTurret(DcMotorEx turretMotor) {
        this.turret = turretMotor;

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }



    public void aimWithOdo(double botErrorDeg, double maxPower) {
        currentPositionTicks = turret.getCurrentPosition();
        currentPositionDegrees = normalize360(minDegrees + (-currentPositionTicks / ticksPerDegree));

        // targetPositionTurretDegrees = normalize360(360 - botErrorDeg);
        targetPositionTurretDegrees = clamp(normalize360(360 - botErrorDeg), safeMinDegrees, safeMaxDegrees);
        targetPositionTurretTicks = -(targetPositionTurretDegrees - minDegrees) * ticksPerDegree;

        turret.setTargetPosition((int) Math.round(targetPositionTurretTicks));
        turret.setPower(Math.abs(maxPower));
    }

    public void stop() {
        turret.setPower(0.0);
    }

    public void resetEncoder() {
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public int getCurrentTicks() {
        return turret.getCurrentPosition();
    }

    public boolean isBusy() {
        return turret.isBusy();
    }


    private int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double normalize360(double deg) {
        deg = deg % 360;
        if (deg < 0) deg += 360;
        return deg;
    }

    public void getTurretTelem(Telemetry t){
        t.addData("curr ticks", (int) Math.round(currentPositionTicks));
        t.addData("curr deg", currentPositionDegrees);
        t.addData("target ticks (raw)", targetPositionTurretTicks);
        t.addData("target ticks (cmd)", (int) Math.round(targetPositionTurretTicks));
        t.addData("target deg", targetPositionTurretDegrees);
    }
}