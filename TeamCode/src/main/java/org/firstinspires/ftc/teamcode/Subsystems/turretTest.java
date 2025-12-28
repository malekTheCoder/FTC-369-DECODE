package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Turret;
@TeleOp(name = "turret test")
public class turretTest extends OpMode {
    Turret turret;
    @Override
    public void init(){
        turret = new Turret(hardwareMap);
    }

    @Override
    public void loop(){

        //right and left bumper for faster movement
        if(gamepad1.right_bumper){
            turret.rotate(true, 50);
        } else if (gamepad1.left_bumper) {
            turret.rotate(false, 50);
        }

        //a and b for fine tuning
        if(gamepad1.aWasPressed()){
            turret.rotate(true, 5);
        }
        else if(gamepad1.bWasPressed()){
            turret.rotate(false, 5);
        }

        telemetry.addData("Turret Current Position", turret.returnPos());
        telemetry.addData("Turret Target Position", turret.returnTarget());

        telemetry.update();
    }
}
