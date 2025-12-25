package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drivetrain {
    private DcMotorEx frontRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;

    public IMU imu;
    double botHeadingIMU;
    private double slowRotationScale = 0.35;

    Drivetrain(HardwareMap hardwareMap){
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

        imu.resetYaw();
        botHeadingIMU = imu.getRobotYawPitchRollAngles().getYaw();
    }

    public void handleDrivetrain(double rightStickX, double x, double y, double rx){
        // drivetrain code to get inputs from controller and call the drive method w/ parameters
        if (Math.abs(rightStickX) < 0.05) rightStickX = 0;
/*
        double rightStickX = gamepad1.right_stick_x
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rx = (gamepad1.right_trigger - gamepad1.left_trigger + (rightStickX * slowRotationScale));
*/
        drive(y, x, rx);
    }
    public void resetIMU(){
        imu.resetYaw();
    }


    public void drive(double forward, double strafe, double rotate){
        // Always field-centric: rotate the joystick vector by -heading
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotatedTheta = AngleUnit.normalizeRadians(theta - heading);

        double f = r * Math.sin(rotatedTheta);
        double s = r * Math.cos(rotatedTheta);

        // Mecanum kinematics
        double frontLeftPower = f + s + rotate;
        double backLeftPower = f - s + rotate;
        double frontRightPower = f - s - rotate;
        double backRightPower = f + s - rotate;

        // Normalize so no value exceeds 1.0
        double max = Math.max(1.0, Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(backLeftPower),
                        Math.max(Math.abs(frontRightPower),
                                Math.abs(backRightPower)))));

        frontLeft.setPower(frontLeftPower / max);
        backLeft.setPower(backLeftPower / max);
        frontRight.setPower(frontRightPower / max);
        backRight.setPower(backRightPower / max);
    }

}
