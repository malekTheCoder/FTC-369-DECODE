package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "motor test")
public class MotorTest extends OpMode {

    private DcMotor motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "turret");

    }

    @Override
    public void loop() {
        motor.setPower(gamepad1.left_stick_y);

    }
}
