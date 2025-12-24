package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {
    DcMotorEx outtake1;
    DcMotorEx outtake2;

    Outtake(HardwareMap hardwareMap){
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
    }


    public void runOuttake(double velocity){
        outtake1.setVelocity(velocity);
        outtake2.setVelocity(velocity);
    }

    public void stopOuttake(){
        outtake1.setVelocity(0);
        outtake2.setVelocity(0);
    }

}
