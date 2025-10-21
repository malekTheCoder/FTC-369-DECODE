package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private double percentPower;
    private double slowRotationScale;



    @Override
    public void init() {
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(DcMotor.class,"fly");
        feeder = hardwareMap.get(CRServo.class, "feeder");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

        imu.resetYaw();

        power = 1;
        percentPower = (power * 100);
        slowRotationScale = 0.5;

        telemetry.addLine("Hardware Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            imu.resetYaw();
        }

        // drivetrain code to get inputs from controller and call the drive method w/ parameters
        // right stick x used for rotating while aiming, scaled down rotation
        double rightStickX = gamepad1.right_stick_x;
        if (Math.abs(rightStickX) < 0.05){
            rightStickX = 0;
        }
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y; //negate it bc its reversed
        double rx = (gamepad1.right_trigger - gamepad1.left_trigger + (rightStickX *slowRotationScale)); // rotation w/ triggers
        drive(y, x, rx);


        handleOuttake();
        handleIntake();
        handleFeeder();

        if (gamepad1.x){
            imu.resetYaw();
        }


        telemetry.addData("Percent Power -> ", percentPower);
        telemetry.update();
    }

    public void drive(double forward, double strafe, double rotate){
        // Always field-centric: rotate the joystick vector by -heading
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
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

        telemetry.addData("heading", Math.toDegrees(heading));
        telemetry.update();
    }

    private void handleFeeder() {
        if (Math.abs(gamepad2.left_stick_y) > 0.05) {
            feeder.setPower(gamepad2.left_stick_y);
        } else {
            feeder.setPower(0);
        }
    }

    private void handleIntake() {
        if (gamepad2.left_trigger > 0.05){
            intake.setPower( 0.5 * gamepad2.left_trigger);
        } else {
            intake.setPower(0);
        }
    }


    private void handleOuttake() {


        if(gamepad2.a){
            power = 0.97;
        } else if (gamepad2.b) {
            power = 0.95;
        } else if (gamepad2.y) {
            power = 0.8;
        } else {
            power = 0;
        }


        outtake.setPower(power);
        telemetry.addData("Percent Power -> ", percentPower);
        telemetry.update();
    }
}
