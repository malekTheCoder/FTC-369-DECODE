package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "flywheel test velocity")
public class FlywheelPID extends OpMode {
    private DcMotorEx fly;
    private double targetVel;

    @Override
    public void init() {
        fly = hardwareMap.get(DcMotorEx.class, "fly");
        fly.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void loop() {

            // D-pad up/down to raise or lower target velocity
        if (gamepad1.dpadUpWasPressed())   targetVel += 100;   // +500 tps
        if (gamepad1.dpadDownWasPressed()) targetVel -= 100;   // -500 tps (reverse if negative)
        if (gamepad1.b)         targetVel = 0;

        targetVel = -gamepad1.left_stick_y*2750;
        fly.setVelocity(targetVel); // ticks per second (negative allowed)

        telemetry.addData("Target (tps)", targetVel);
        telemetry.addData("Actual (tps)", fly.getVelocity());
        telemetry.update();


    }
}
