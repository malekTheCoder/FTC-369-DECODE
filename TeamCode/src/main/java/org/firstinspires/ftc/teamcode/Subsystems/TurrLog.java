package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;

public class TurrLog {

    private static final int TICKS_PER_TURRET_REV = -853; // -853 sp

    private static final double TICKS_PER_DEG = TICKS_PER_TURRET_REV / 320.0;

    private static final double closedZone = 20;

    private boolean inReach;

    int targetTicks;
    double targetDeg;


    private final DcMotorEx turret;

    public TurrLog(DcMotorEx turret) {
        this.turret = turret;

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0);
    }


    public void aim(double botErrorDeg, Telemetry t) {

        // botErrorDeg is in [-180, 180]. 0 means the robot is facing the goal.
        // We want the turret to take the SHORT way to the same physical direction.

        // Current turret angle (continuous, not wrapped)
        int currentTicks = turret.getCurrentPosition();
        double currentDeg = currentTicks / TICKS_PER_DEG; // 20 for starting

        // Desired turret angle from your vision/aim math.
        // Keep this as just botErrorDeg (no +180) so 0 deg -> 0 ticks.
        double desiredDeg = botErrorDeg;

        // Compute the smallest rotation needed to go from currentDeg -> desiredDeg (useful for telemetry)
        double deltaDeg = wrapTo180(desiredDeg - currentDeg);

        // Choose a safe turret target that NEVER enters the closed wiring zone around robot-front (0°).
        // NOTE: currentDeg is "mechanical degrees" where 0° == physical +20°.
        targetDeg = chooseSafeTarget(currentDeg, desiredDeg);
        targetTicks = (int) Math.round(targetDeg * TICKS_PER_DEG);

        // Update inReach after choosing the target
        inReach = isInReach();



        turret.setTargetPosition(targetTicks);
        turret.setPower(0.1);



        t.addData("botErrorDeg", botErrorDeg);
        t.addData("in reach", inReach);
        t.addData("currentDeg", currentDeg);
        t.addData("desiredDeg", desiredDeg);
        t.addData("deltaDeg(short)", deltaDeg);
        t.addData("targetDeg", targetDeg);
        t.addData("Target Ticks", targetTicks);
    }

    private double chooseSafeTarget(double currentDeg, double desiredDeg) {
        // Mechanical degrees: encoder 0 ticks == physical +20°
        final double OFFSET_DEG = closedZone;   // 20°
        final double MIN_PHYS = closedZone;     // 20°
        final double MAX_PHYS = 360.0 - closedZone; // 340°

        // Convert current mechanical -> physical
        double currentPhysDeg = currentDeg + OFFSET_DEG;

        // Convert desired from [-180,180] -> [0,360)
        double desiredPhysDeg = desiredDeg;
        if (desiredPhysDeg < 0) desiredPhysDeg += 360.0;

        // If desired is in the forbidden band near 0°, pick the closest safe boundary
        if (desiredPhysDeg < MIN_PHYS || desiredPhysDeg > MAX_PHYS) {
            double candA = MIN_PHYS; // +20°
            double candB = MAX_PHYS; // +340°

            double distA = Math.abs(candA - currentPhysDeg);
            double distB = Math.abs(candB - currentPhysDeg);

            desiredPhysDeg = (distA <= distB) ? candA : candB;
        }

        // Clamp to the allowed physical arc just in case
        if (desiredPhysDeg < MIN_PHYS) desiredPhysDeg = MIN_PHYS;
        if (desiredPhysDeg > MAX_PHYS) desiredPhysDeg = MAX_PHYS;

        // Convert physical -> mechanical target degrees (what RUN_TO_POSITION should use)
        return desiredPhysDeg - OFFSET_DEG;
    }


    /** Wraps an angle to the range (-180, 180]. This gives you the shortest turn direction. */
    private static double wrapTo180(double deg) {
        while (deg <= -180) deg += 360;
        while (deg > 180) deg -= 360;
        return deg;
    }

    public boolean isInReach(){
        // targetDeg is mechanical degrees where 0° == physical +20°
        double targetPhysDeg = targetDeg + closedZone;
        return (targetPhysDeg >= closedZone) && (targetPhysDeg <= (360.0 - closedZone));
    }
}