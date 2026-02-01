package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp(name = "turret test encoder ticks")
public class TurretTest extends OpMode {


    // -853 spinning cw from the front

    DcMotorEx turret;
    @Override
    public void init(){
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop(){

        //right and left bumper for faster movement
        turret.setPower(gamepad1.left_stick_x);

        //a and b for fine tuning
        if(gamepad1.a){
            turret.setPower(-.5);
        }
        else if(gamepad1.b){
            turret.setPower(.5);
        }
        else{
            turret.setPower(0);
        }

        telemetry.addData("Turret Current Position", turret.getCurrentPosition());

        telemetry.addData("Turret Target Position", turret.getTargetPosition());

        telemetry.update();
    }
}