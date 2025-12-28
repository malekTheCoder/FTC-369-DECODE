package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    DcMotorEx turret;
    private int maxTarget = 1800; //placeholder value for maximum target position
    private int minimumTarget = 1800; //placeholder value for minimum target position
    private int ticksPerDegree = 10; //placeholder value for now, change when receiving correct value
    Turret(HardwareMap hardwareMap){
        turret = hardwareMap.get(DcMotorEx.class, "turret");
    }

    public void trackAngle(int target){//target is the target position in degrees
        int targetPos = target*ticksPerDegree;
        if(targetPos < maxTarget && targetPos > minimumTarget) {
            turret.setTargetPosition(targetPos);
        }
        else if(targetPos > maxTarget){
            turret.setTargetPosition(minimumTarget);
        }
        else{
            turret.setTargetPosition(maxTarget);
        }
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
