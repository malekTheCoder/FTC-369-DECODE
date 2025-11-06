package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Final V1 Bot Teleop")
public class FinalTeleopV1Bot extends OpMode {

    private Limelight3A limelight;
    private DcMotorEx frontRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;

    private DcMotor intake;
    private DcMotorEx fly;
    private CRServo feeder;
    private IMU imu;

    YawPitchRollAngles orientation;
    LLResult llResult;


    private double targetVel;


    private BasicPID turnPID;
    private PIDCoefficients turnPIDCoeffs;

    private double Kp = 1.25; // one of the main values, increase if it goes slowly or stops early and reduce slightly if it overshoots and oscillates
    private double Ki = 0.0001; // keep 0 for aiming
    private double Kd = 0.1; // start with 0.1, if its overshooting increase a little bit (by 0.02-0.05), if it feels twitchy reduce it a little bit

    private double slowRotationScale = 0.35;

    double botHeadingIMU;
    double botHeadingAtCapture;

    double desiredHeading;
    double currentHeading;

    boolean aiming;

    double turnError;
    double turnCommand = 0;


    double targetOffsetAngle_Vertical;
    double limelightMountAngleDegrees = 15;
    double limelightLensHeightInches = 14.5;
    double goalHeightInches = 29.5;

    double angleToGoalDegrees;
    double angleToGoalRadians;
    double distanceFromLimelightToGoalInches;

    private double ty;
    private double tx;

    private double flyMultiplier;


    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        initDriveAndIMU();
        initFlyWheel();

        intake = hardwareMap.get(DcMotor.class, "intake");
        feeder = hardwareMap.get(CRServo.class, "feeder");

        turnPIDCoeffs = new PIDCoefficients(Kp, Ki, Kd);
        turnPID = new BasicPID(turnPIDCoeffs);

        limelight.start();
        telemetry.addLine("Hardware Initialized!");

    }



    @Override
    public void loop() {
        updateLimelightInfo();
        handleFlywheel();
        handleFeeder();
        handleIntake();


        botHeadingIMU = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


        if (gamepad1.aWasPressed()){
            // if valid detection
            if (llResult != null && llResult.isValid()){
                aiming = true;
                botHeadingAtCapture = botHeadingIMU;
                desiredHeading = AngleUnit.normalizeRadians(botHeadingAtCapture - AngleUnit.normalizeRadians(Math.toRadians(tx)));
            }
        } else if (gamepad1.aWasReleased()){
            aiming = false;
        }


        if (gamepad1.a){
            aim();
        } else {
            handleDrivetrain();
        }


        telemetry.addData("Target (tps)", targetVel);
        telemetry.addData("Actual (tps)", fly.getVelocity());

        if (llResult != null && llResult.isValid()){
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("Bot pose", botPose.toString());
            telemetry.addData("Yaw", botPose.getOrientation().getYaw());
            telemetry.addData("Distance", distanceFromLimelightToGoalInches);
        }


        telemetry.update();
    }

    private void handleFlywheel() {
        targetVel = shooterModel(distanceFromLimelightToGoalInches);

        if (gamepad1.yWasPressed()){
            if (flyMultiplier == 1){
                flyMultiplier = 0;
            }
            else{
                flyMultiplier = 1;
            }
        }
        fly.setVelocity(targetVel); // ticks per second (negative allowed)
    }

    private double shooterModel (double distanceInches){
        return 0.0; // add regression here to return the velocity needed given the distance
    }

    private void updateLimelightInfo() {
        orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        llResult = limelight.getLatestResult();
        ty = llResult.getTy();
        tx = llResult.getTx();
        targetOffsetAngle_Vertical = ty;
        angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
        distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }


    private void handleFeeder() {
        if (gamepad1.right_stick_y > 0.05) {
            feeder.setPower(gamepad2.left_stick_y);
        } else {
            feeder.setPower(0);
        }

    }

    private void handleIntake(){
        if (gamepad1.right_stick_y < - 0.05){
            intake.setPower(gamepad1.right_stick_y);
        } else {
            intake.setPower(0);
        }
    }
    private void handleDrivetrain() {

        if (gamepad1.x){
            imu.resetYaw();
        }

        // drivetrain code to get inputs from controller and call the drive method w/ parameters
        double rightStickX = gamepad1.right_stick_x;
        if (Math.abs(rightStickX) < 0.05) rightStickX = 0;


        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rx = (gamepad1.right_trigger - gamepad1.left_trigger + (rightStickX * slowRotationScale));

        drive(y, x, rx);
    }

    private void aim() {
        if(!aiming) return;
        currentHeading = botHeadingIMU;
        turnError = AngleUnit.normalizeRadians(desiredHeading - currentHeading);
        turnCommand = turnPID.calculate(0.0, turnError);

        if (turnCommand > 1) turnCommand = 1;
        if (turnCommand < -1) turnCommand = -1;

        drive(0,0, turnCommand);
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

    private void initDriveAndIMU() {
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


    private void initFlyWheel() {
        fly = hardwareMap.get(DcMotorEx.class, "fly");
        fly.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

}
