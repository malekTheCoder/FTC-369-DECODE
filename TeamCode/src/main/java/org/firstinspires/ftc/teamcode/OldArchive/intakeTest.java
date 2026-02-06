package org.firstinspires.ftc.teamcode.OldArchive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
