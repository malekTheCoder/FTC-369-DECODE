package org.firstinspires.ftc.teamcode.teleop;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Disabled
@Config
@TeleOp(name = "tune turning")
public class TuneTurn extends OpMode {
    private DcMotorEx frontRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;

    private IMU imu;

    private BasicPID turnPID;
    private PIDCoefficients turnPIDCoeffs;

    public static double Kp = 0.0; // one of the main values, increase if it goes slowly or stops early and reduce slightly if it overshoots and oscillates
    public static double Ki = 0.0; // keep 0 for aiming
    public static double Kd = 0.0; // start with 0.1, if its overshooting increase a little bit (by 0.02-0.05), if it feels twitchy reduce it a little bit

    //
    private double previousKp = Kp;
    private double previousKi = Ki;
    private double previousKd = Kd;

    private double targetBearing = 0.0;

    double currentHeading;
    double error;
    double turn = 0;




    private double slowRotationScale = 0.5;


    @Override
    public void init() {
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

        turnPIDCoeffs = new PIDCoefficients(Kp, Ki, Kd);
        turnPID = new BasicPID(turnPIDCoeffs);

        // so error=0 on init
        targetBearing = 0.0; // radians, relative to resetYaw()
        currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        error = AngleUnit.normalizeRadians(targetBearing - currentHeading);

    }

    @Override
    public void loop() {
        if (gamepad1.a){
            aim();
        } else {
            handleDrivetrain();
        }



        telemetry.addData("target (deg)", Math.toDegrees(targetBearing));
        telemetry.addData("heading (deg)", Math.toDegrees(currentHeading));
        telemetry.addData("error (deg)",   Math.toDegrees(error));
        telemetry.addData("turn", turn);
        telemetry.update();
    }

    private void aim() {
        if (gamepad1.dpadLeftWasPressed()) targetBearing -= Math.toRadians(45);
        if (gamepad1.dpadRightWasPressed()) targetBearing += Math.toRadians(45);
        if (gamepad1.y){
            imu.resetYaw();
            targetBearing = 0;
        }

         currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
         error = AngleUnit.normalizeRadians(targetBearing - currentHeading);

        // Rebuild the PID controller only when any tuning value has changed (for live Dashboard edits)
        boolean pidValuesChanged = (Kp != previousKp) || (Ki != previousKi) || (Kd != previousKd);

        if (pidValuesChanged) {
            previousKp = Kp;
            previousKi = Ki;
            previousKd = Kd;

            turnPIDCoeffs = new PIDCoefficients(Kp, Ki, Kd);
            turnPID = new BasicPID(turnPIDCoeffs);
        }

         turn = turnPID.calculate(error, 0);
        if (turn > 1) turn = 1;
        if (turn <-1) turn = -1;

        drive(0,0,turn);


    }

    private void handleDrivetrain() {
        if (gamepad1.x){
            imu.resetYaw();
        }

        // drivetrain code to get inputs from controller and call the drive method w/ parameters
        double rightStickX = gamepad1.right_stick_x;
        if (Math.abs(rightStickX) < 0.05){
            rightStickX = 0;
        }
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rx = (gamepad1.right_trigger - gamepad1.left_trigger + (rightStickX* slowRotationScale)); // rotation w/ triggers
        drive(y, x, rx);
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
}
