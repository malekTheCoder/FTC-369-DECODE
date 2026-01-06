package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;

public class TurretLogic {


    private static final double TICKS_PER_TURRET_REV = 1930; // placeholder
    private static final double TICKS_PER_DEG = TICKS_PER_TURRET_REV / 320;

    private int lastTarget;

    private static final double MIN_DEG = -360;
    private static final double MAX_DEG =  360;

    private final DcMotorEx turret;

    public TurretLogic(DcMotorEx turret) {
        this.turret = turret;

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }



    // new turret limits 340-20 degrees is the 40 degree deadzone centered at the front of the bot
    public void aim(double botErrorDeg, Telemetry t) {

        // invert sign (your convention)
        double turretDeg = botErrorDeg;

        // HARD SAFETY LIMITS

        if (turretDeg > 360) {
            turretDeg -= 360;
            t.addLine("HARD STOP ACTIVATED");
        }

        if (turretDeg < -360) {
            turretDeg += 360;
            t.addLine("HARD STOP ACTIVATED");
        }








        int targetTicks = (int) (turretDeg * TICKS_PER_DEG);

        lastTarget = targetTicks;

        turret.setTargetPosition(targetTicks);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);

        t.addData("Target Ticks", targetTicks);
        t.addData("turret deg", turretDeg);



    }
}