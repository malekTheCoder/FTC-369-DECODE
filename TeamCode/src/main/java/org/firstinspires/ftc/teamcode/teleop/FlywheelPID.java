package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class FlywheelPID extends OpMode {
    private DcMotorEx fly;

    @Override
    public void init() {
        fly = hardwareMap.get(DcMotorEx.class, "fly");
        fly.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void loop() {
        fly.setVelocity(1);

    }
}
