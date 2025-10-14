package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// saved copy

@TeleOp(name = "prototype bot")
public class PrototypeBotTest extends OpMode{

    private DcMotor outtake;

    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    private DcMotor intake;

    private CRServo feeder;

    private IMU imu;

    private double power;

    @Override
    public void init() {
        outtake = hardwareMap.get(DcMotor.class,"fly");

        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        intake = hardwareMap.get(DcMotor.class, "intake");

        feeder = hardwareMap.get(CRServo.class, "feeder");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

        imu.resetYaw();

        power = 1;

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);



        telemetry.addLine("Hardware Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        handleDrivetrain();
        handleOuttake();
        handleIntake();
        handleFeeder();

        if (gamepad1.x){
            imu.resetYaw();
        }

        telemetry.update();
    }

    private void handleFeeder() {
        if (Math.abs(gamepad2.left_stick_y) > 0.1) {
            feeder.setPower(gamepad2.left_stick_y);
        } else {
            feeder.setPower(0);
        }
    }

    private void handleIntake() {
        if (gamepad2.left_trigger > 0.1){
            intake.setPower(gamepad2.left_trigger);
        } else {
            intake.setPower(0);
        }
    }

    private void handleDrivetrain() {
        
        // Get drive inputs (negated Y because joystick Y is reversed)
        double x = -gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;  // Negated to match standard coordinate system
        double rx = (gamepad1.right_trigger - gamepad1.left_trigger);
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Apply field centric transformation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Calculate motor powers using transformed inputs
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        telemetry.addData("imu reading", Math.toDegrees(botHeading));
    }

    

    private void handleOuttake() {


        if(gamepad2.a){
            power = 1;
        } else if (gamepad2.b) {
            power = 0.95;
        } else if (gamepad2.y) {
            power = 0.8;
        } else {
            power = 0;
        }


        outtake.setPower(power);
    }
}
