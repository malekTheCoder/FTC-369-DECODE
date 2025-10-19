package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@TeleOp(name = "turn bot test")
public class AimBot extends OpMode {
    private DcMotorEx frontRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;

    private IMU imu;

    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;

    private PIDEx turnPID;
    private PIDCoefficientsEx turnPIDCoeffs;


    public static double Kp = 0.5; // one of the main values, increase if it goes slowly or stops early and reduce slightly if it overshoots and oscillates
    public static double Ki = 0.0; // keep 0 for aiming
    public static double Kd = 0.10; // start with 0.1, if its overshooting increase a little bit (by 0.02-0.05), if it feels twitchy reduce it a little bit
    public static double integralSumMax = 0.5;
    public static double stabilityThreshold = 0.5;
    public static double lowPassGain = 0.9;

    private double prevKp = Kp, prevKi = Ki, prevKd = Kd;
    private double prevInt = integralSumMax, prevStab = stabilityThreshold, prevLP = lowPassGain;

    private double targetBearing = 0.0;

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

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                //.setLensIntrinsics()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(640,480))
                .build();



        turnPIDCoeffs = new PIDCoefficientsEx(Kp, Ki, Kd, integralSumMax, stabilityThreshold, lowPassGain);
        turnPID = new PIDEx(turnPIDCoeffs);
    }

    @Override
    public void loop() {
        if (Kp != prevKp || Ki != prevKi || Kd != prevKd || integralSumMax != prevInt || stabilityThreshold != prevStab || lowPassGain != prevLP) {
            prevKp = Kp; prevKi = Ki; prevKd = Kd;
            prevInt = integralSumMax; prevStab = stabilityThreshold; prevLP = lowPassGain;

            turnPIDCoeffs = new PIDCoefficientsEx(Kp, Ki, Kd, integralSumMax, stabilityThreshold, lowPassGain);
            turnPID = new PIDEx(turnPIDCoeffs);
        }


        if (gamepad1.a){
            handleAimBot();
        } else {
            handleDrivetrain();
        }



    }

    private void handleAimBot() {
        double bearing = 0;
        double turnCommand = 0;

        if (gamepad1.a){
            List<AprilTagDetection> detectionsList = tagProcessor.getFreshDetections();
            if (detectionsList != null && !detectionsList.isEmpty()){
                AprilTagDetection tag = detectionsList.get(0);

                bearing = tag.ftcPose.bearing;
                turnCommand = turnPID.calculate(targetBearing, bearing);

                if (turnCommand > 1) turnCommand = 1;
                if (turnCommand < -1) turnCommand = -1;

                drive(0,0, turnCommand);

                telemetry.addData("AIM: id", tag.id);
                telemetry.addData("AIM: bearing (deg)", "%.1f", Math.toDegrees(bearing));
                telemetry.addData("AIM: turnCmd", "%.3f", turnCommand);
                telemetry.update();
            } else {
                telemetry.addLine("AIM: no tag");
                telemetry.update();
            }
        }
    }

    private void handleDrivetrain() {

        if (gamepad1.x){
            imu.resetYaw();
        }

        // drivetrain code to get inputs from controller and call the drive method w/ parameters
        double leftStickX = gamepad1.left_stick_x;
        if (Math.abs(leftStickX) < 0.05){
            leftStickX = 0;
        }
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rx = (gamepad1.right_trigger - gamepad1.left_trigger + (leftStickX* slowRotationScale)); // rotation w/ triggers
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
