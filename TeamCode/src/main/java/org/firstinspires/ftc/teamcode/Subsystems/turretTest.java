package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Turret;
@TeleOp(name = "turret test")
public class turretTest extends OpMode {
    DcMotorEx turret;
    @Override
    public void init(){
        turret = hardwareMap.get(DcMotorEx.class, "turret");
    }

    @Override
    public void loop(){

        //right and left bumper for faster movement
        turret.setPower(gamepad1.left_stick_x);

        //a and b for fine tuning
        if(gamepad1.a){
            turret.setPower(-.1);
        }
        else if(gamepad1.b){
            turret.setPower(.1);
        }
        else{
            turret.setPower(0);
        }

        telemetry.addData("Turret Current Position", turret.getCurrentPosition());

        telemetry.addData("Turret Target Position", turret.getTargetPosition());

        telemetry.update();
    }
}
