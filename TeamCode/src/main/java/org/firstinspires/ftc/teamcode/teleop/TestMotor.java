package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "Test motors")
public class TestMotor extends OpMode {
    private DcMotor motor;


    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class,"frontLeft");
    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()){
            motor = hardwareMap.get(DcMotor.class,"frontLeft");
        } else if (gamepad1.bWasPressed()) {
            motor = hardwareMap.get(DcMotor.class,"frontRight");
        } else if (gamepad1.aWasPressed()) {
            motor = hardwareMap.get(DcMotor.class,"backRight");
        } else if (gamepad1.xWasPressed()) {
            motor = hardwareMap.get(DcMotor.class,"backLeft");
        }

        motor.setPower(-gamepad1.left_stick_y);
    }

}
