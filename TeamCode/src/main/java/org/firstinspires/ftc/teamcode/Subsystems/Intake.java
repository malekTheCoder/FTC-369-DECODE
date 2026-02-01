package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intake;

    public Intake(HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void runIntake(double power){
        intake.setPower(power);
    }

    public void stopIntake(){
        intake.setPower(0);
    }
}
