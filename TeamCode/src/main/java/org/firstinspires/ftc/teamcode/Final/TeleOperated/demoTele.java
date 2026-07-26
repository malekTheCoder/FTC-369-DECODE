package org.firstinspires.ftc.teamcode.Final.TeleOperated;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class demoTele extends OpMode {
    private DcMotor intake;
    private Gamepad gp;
    @Override
    public void init(){
        intake = hardwareMap.get(DcMotor.class, "intake");
    }
    @Override
    public void loop(){
        if(gp.)
    }
}
