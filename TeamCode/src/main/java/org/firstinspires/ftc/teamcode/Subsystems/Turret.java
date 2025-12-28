package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    DcMotorEx turret;
    //Max target and minimum target might not be needed
    private int maxTarget = 1800; //placeholder value for maximum target position
    private int minimumTarget = 1800; //placeholder value for minimum target position
    private int ticksPerDegree = 10; //placeholder value for now, change when receiving correct value
    Turret(HardwareMap hardwareMap){
        turret = hardwareMap.get(DcMotorEx.class, "turret");
    }

    public void trackAngle(int target){//target is the target position in degrees
        int targetPos = (target+180)*ticksPerDegree;

        turret.setTargetPosition(targetPos);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
