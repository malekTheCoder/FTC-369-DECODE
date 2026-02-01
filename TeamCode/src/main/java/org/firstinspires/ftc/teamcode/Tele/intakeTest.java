package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;

@TeleOp(name = "intake test")
public class intakeTest extends OpMode {
    DcMotor intake;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
    }

    @Override
    public void loop() {
        intake.setPower(.8);
    }
}
