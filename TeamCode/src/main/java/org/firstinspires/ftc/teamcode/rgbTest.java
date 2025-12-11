package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "RGB test")
public class rgbTest extends OpMode{
    private Servo rgbLight;
    private double rgbLightValue;

    @Override
    public void init(){
        rgbLight = hardwareMap.get(Servo.class, "rgb");
    }
    @Override
    public void loop(){
        //.25 and below is blank
        //.25 is red
        //.7 and above is white
        //red - orange - yellow - green - blue - violet - white
        if(gamepad1.dpad_down){
            rgbLightValue-=.001;
        }
        if(gamepad1.dpad_up){
            rgbLightValue+=.001;
        }
        rgbLight.setPosition(rgbLightValue);

        telemetry.addData("rgbValue", rgbLightValue);
        telemetry.update();
    }
}
