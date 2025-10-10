package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;



@TeleOp(name = "basic teleop")
public class BasicTeleop extends OpMode{

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;


    private DcMotor intake;
    private DcMotor outtake;

    @Override
    public void init() {
        backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        backRight = hardwareMap.get(DcMotor.class,"backRight");
        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        frontRight = hardwareMap.get(DcMotor.class,"frontRight");

        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(DcMotor.class,"outtake");

        telemetry.addLine("Hardware Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        intake();
        drivetrain();
    }
    private void intake(){

        if(gamepad1.left_bumper){
            intake.setPower(1);
        }
        else if (gamepad1.right_bumper){
            intake.setPower(-1);
        }
        else{
            intake.setPower(0);
        }
        outtake.setPower(gamepad1.right_trigger);
    }
    private void drivetrain(){

        if(gamepad1.left_stick_x < -.2 || gamepad1.left_stick_x > .2){
            frontLeft.setPower(gamepad1.left_stick_x);
            frontRight.setPower(-gamepad1.left_stick_x);
            backLeft.setPower(-gamepad1.left_stick_x);
            backRight.setPower(gamepad1.left_stick_x);
        }

        if(gamepad1.left_stick_y < -.2 || gamepad1.left_stick_y > .2){
            frontLeft.setPower(gamepad1.left_stick_y);
            frontRight.setPower(gamepad1.left_stick_y);
            backLeft.setPower(gamepad1.left_stick_y);
            backRight.setPower(gamepad1.left_stick_y);
        }

        if(gamepad1.right_stick_x< -.2 || gamepad1.right_stick_x>.2){
            frontLeft.setPower(gamepad1.left_stick_y);
            frontRight.setPower(-gamepad1.left_stick_y);
            backLeft.setPower(gamepad1.left_stick_y);
            backRight.setPower(-gamepad1.left_stick_y);
        }
    }
}
