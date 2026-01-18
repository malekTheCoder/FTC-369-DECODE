package org.firstinspires.ftc.teamcode.teleop;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Disabled
@TeleOp(name = "shoot test")
public class shootTest extends OpMode {

    DcMotorEx fly1;
    DcMotorEx fly2;

    DcMotor transfer;

    @Override
    public void init() {
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
        transfer = hardwareMap.get(DcMotor.class, "intake");

        fly2.setDirection(DcMotorSimple.Direction.REVERSE);

        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fly1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fly2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        if(gamepad1.dpad_left){
            fly1.setPower(.7);
            fly2.setPower(.7);
        }
        else if(gamepad1.dpad_right){
            fly1.setPower(.8);
            fly2.setPower(.8);
        }
        else if(gamepad1.dpad_down){
            fly1.setPower(.6);
            fly2.setPower(.6);
        }
        else if(gamepad1.dpad_up){
            fly1.setPower(.9);
            fly2.setPower(.9);
        }
        else{
            fly1.setPower(0);
            fly2.setPower(0);
        }

        transfer.setPower(gamepad2.left_stick_y);

    }
}