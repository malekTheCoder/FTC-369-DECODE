package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drivetrain {
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    public IMU imu;
    double botHeadingIMU;
    private double slowRotationScale = 0.35;

    private double pinpointHeadingAdjuster = 0;

    public Drivetrain(HardwareMap hardwareMap){
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        //not using control hub imu for now
//        imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP));
//
//        imu.initialize(parameters);
//
//        imu.resetYaw();
//
//        botHeadingIMU = imu.getRobotYawPitchRollAngles().getYaw();
    }

    public Drivetrain(HardwareMap hardwareMap, double pinpointHeadingAdjuster) {
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.pinpointHeadingAdjuster = pinpointHeadingAdjuster;

    }

//

    public void handleDrivetrainWithPinpoint(Gamepad gp1, double heading){
        // drivetrain code to get inputs from controller and call the drive method w/ parameters

        double rightStickX = gp1.right_stick_x;
        if (Math.abs(rightStickX) < 0.05) rightStickX = 0;
        double x = gp1.left_stick_x;
        double y = -gp1.left_stick_y;
        double rx = (gp1.right_trigger - gp1.left_trigger + (rightStickX * slowRotationScale));

        driveWithPinpointHelper(y, x, rx, heading);
    }


    public void driveWithPinpointHelper(double forward, double strafe, double rotate, double headingPinpoint){
        // Always field-centric: rotate the joystick vector by -heading
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        double heading = Math.toRadians(AngleUnit.normalizeDegrees(headingPinpoint + pinpointHeadingAdjuster));
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
    public void driveRobotCentric(double forward, double strafe, double rotate) {

        // Mecanum kinematics (robot frame)
        double frontLeftPower  = forward + strafe + rotate;
        double backLeftPower   = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower  = forward + strafe - rotate;

        // Normalize so nothing exceeds |1.0|
        double max = Math.max(1.0,
                Math.max(Math.abs(frontLeftPower),
                        Math.max(Math.abs(backLeftPower),
                                Math.max(Math.abs(frontRightPower),
                                        Math.abs(backRightPower)))));

        frontLeft.setPower(frontLeftPower / max);
        backLeft.setPower(backLeftPower / max);
        frontRight.setPower(frontRightPower / max);
        backRight.setPower(backRightPower / max);
    }

    public void setPinpointHeadingAdjuster(double pinpointHeadingAdjuster) {
        this.pinpointHeadingAdjuster = pinpointHeadingAdjuster;
    }

    public double getPinpointHeadingAdjuster(){
        return  pinpointHeadingAdjuster;
    }


    public void setZeroPower(){
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }





   // public void handleDrivetrain(Gamepad gp1){
//        // drivetrain code to get inputs from controller and call the drive method w/ parameters
//
//        double rightStickX = gp1.right_stick_x;
//        if (Math.abs(rightStickX) < 0.05) rightStickX = 0;
//        double x = gp1.left_stick_x;
//        double y = -gp1.left_stick_y;
//        double rx = (gp1.right_trigger - gp1.left_trigger + (rightStickX * slowRotationScale));
//
//        drive(y, x, rx);
//    }

    //    public void drive(double forward, double strafe, double rotate){
//        // Always field-centric: rotate the joystick vector by -heading
//        double theta = Math.atan2(forward, strafe);
//        double r = Math.hypot(strafe, forward);
//
//        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//        double rotatedTheta = AngleUnit.normalizeRadians(theta - heading);
//
//        double f = r * Math.sin(rotatedTheta);
//        double s = r * Math.cos(rotatedTheta);
//
//        // Mecanum kinematics
//        double frontLeftPower = f + s + rotate;
//        double backLeftPower = f - s + rotate;
//        double frontRightPower = f - s - rotate;
//        double backRightPower = f + s - rotate;
//
//        // Normalize so no value exceeds 1.0
//        double max = Math.max(1.0, Math.max(Math.abs(frontLeftPower),
//                Math.max(Math.abs(backLeftPower),
//                        Math.max(Math.abs(frontRightPower),
//                                Math.abs(backRightPower)))));
//
//        frontLeft.setPower(frontLeftPower / max);
//        backLeft.setPower(backLeftPower / max);
//        frontRight.setPower(frontRightPower / max);
//        backRight.setPower(backRightPower / max);
//    }

}
