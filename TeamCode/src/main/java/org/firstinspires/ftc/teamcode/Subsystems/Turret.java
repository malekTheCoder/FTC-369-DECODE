package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    DcMotorEx turret;
    //Max target and minimum target might not be needed
    //full revolution is 1930 ticks
    private int maxTarget = 1800; //placeholder value for maximum target position
    private int minimumTarget = 1800; //placeholder value for minimum target position
    private int ticksPerDegree = 1930/360; //placeholder value for now, change when receiving correct value
    Turret(HardwareMap hardwareMap){
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void trackAngle(int target){//target is the target position in degrees
        int targetPos = (target+180)*ticksPerDegree;

        turret.setTargetPosition(targetPos);

        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(.6);
    }
    public double returnPos(){
        return turret.getCurrentPosition();
    }
    public double returnTarget(){
        return turret.getTargetPosition();
    }

    public void rotate(boolean direction, int step){
        if(direction){
            turret.setTargetPosition(turret.getCurrentPosition()+step);
        }
        else{
            turret.setTargetPosition(turret.getCurrentPosition()-step);
        }

        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
