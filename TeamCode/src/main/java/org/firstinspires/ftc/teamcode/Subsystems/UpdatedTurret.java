package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

    public UpdatedTurret(DcMotorEx turret) {
        this.turret = turret;

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        inDeadZone = true;
    }

    public void update (double botErrorDeg, Telemetry t){
        currentPositionTicks = turret.getCurrentPosition();
        currentPositionDegrees = normalize360(minDegrees + (-currentPositionTicks / ticksPerDegree));

        // targetPositionTurretDegrees = normalize360(360 - botErrorDeg);
        targetPositionTurretDegrees = clamp(normalize360(360 - botErrorDeg), safeMinDegrees, safeMaxDegrees);
        targetPositionTurretTicks = -(targetPositionTurretDegrees - minDegrees) * ticksPerDegree;

//        if (targetPositionTurretTicks > maxSafeTicks){
//            targetPositionTurretTicks = maxSafeTicks;
//            targetPositionTurretDegrees = minDegrees - (targetPositionTurretTicks / ticksPerDegree);
//
//        } else if (targetPositionTurretTicks < minSafeTicks){
//            targetPositionTurretTicks = minSafeTicks;
//            targetPositionTurretDegrees = minDegrees - (targetPositionTurretTicks / ticksPerDegree);
//        }



        t.addData("bot error deg", botErrorDeg);
        t.addData("curr ticks", currentPositionTicks);
        t.addData("curr deg", currentPositionDegrees);
        t.addLine("---------------");
        t.addData("target ticks", targetPositionTurretTicks);
        t.addData("target deg", targetPositionTurretDegrees);
        t.addLine("---------------");
        t.addData("in deadzone", inDeadZone);
        t.addData("on target", isOnTarget(5));
        t.addLine("---------------");
        t.update();

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


    public void aim(double power){
        turret.setTargetPosition((int)targetPositionTurretTicks);
        turret.setPower(power);
    }

    public boolean isOnTarget(double degTolerance){
        double tickDiff = Math.abs(turret.getCurrentPosition() - targetPositionTurretTicks);
        return (tickDiff/ticksPerDegree) < degTolerance;
    }

}
