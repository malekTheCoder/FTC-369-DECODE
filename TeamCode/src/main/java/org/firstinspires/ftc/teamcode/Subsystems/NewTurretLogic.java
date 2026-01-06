//package org.firstinspires.ftc.teamcode.Subsystems;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.opencv.core.Mat;
//
//public class TurrLog {
//
//    private static final int TICKS_PER_TURRET_REV = 853; // -853 sp
//
//    private static final double TICKS_PER_DEG = TICKS_PER_TURRET_REV / 320.0;
//
//    private static final double closedZone = 20;
//
//    private boolean inReach;
//
//    int targetTicks;
//    double targetDeg;
//
//
//    private final DcMotorEx turret;
//
//    public TurrLog(DcMotorEx turret) {
//        this.turret = turret;
//
//        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        turret.setPower(0);
//    }
//
//
//    public void aim(double botErrorDeg, Telemetry t) {
//
//        // botErrorDeg is in [-180, 180]. 0 means the robot is facing the goal.
//        // We want the turret to take the SHORT way to the same physical direction.
//
//        // Current turret angle (continuous, not wrapped)
//        int currentTicks = turret.getCurrentPosition();
//        double currentDeg = currentTicks / TICKS_PER_DEG; // 20 for starting
//
//        // Desired turret angle from your vision/aim math.
//        // Keep this as just botErrorDeg (no +180) so 0 deg -> 0 ticks.
//        double desiredDeg = botErrorDeg;
//
//        // Compute the smallest rotation needed to go from currentDeg -> desiredDeg
//        double deltaDeg = wrapTo180(desiredDeg - currentDeg);
//
//        // New target is "current + smallest delta" so it never takes the long route
//        // targetDeg = currentDeg + deltaDeg;
//        targetDeg = chooseSafeTarget(currentDeg, desiredDeg);
//        targetTicks = (int) Math.round(targetDeg * TICKS_PER_DEG);
//
//
//        turret.setTargetPosition(targetTicks);
//        turret.setPower(0.1);
//
//
//
//        t.addData("botErrorDeg", botErrorDeg);
//        t.addData("in reach", inReach);
//        t.addData("currentDeg", currentDeg);
//        t.addData("desiredDeg", desiredDeg);
//        t.addData("deltaDeg(short)", deltaDeg);
//        t.addData("targetDeg", targetDeg);
//        t.addData("Target Ticks", targetTicks);
//    }
//
//    private double chooseSafeTarget(double currentDeg, double desiredDeg) {
//    }
//
//
//    /** Wraps an angle to the range (-180, 180]. This gives you the shortest turn direction. */
//    private static double wrapTo180(double deg) {
//        while (deg <= -180) deg += 360;
//        while (deg > 180) deg -= 360;
//        return deg;
//    }
//
//    public boolean isInReach(){
//        double value = Math.abs(targetDeg);
//        return (value > 22 && value < 338);
//    }
//}