package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "flywheel test velocity")
public class FlywheelPID extends OpMode {
    private DcMotorEx frontRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;

    private DcMotor intake;
    private DcMotor outtake;
    private CRServo feeder;
    private IMU imu;

    private DcMotorEx fly;
    private double targetVel;

    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;

    double bearing = 0;

     /*
        intrinsics found online for the gobuilda usb camera with 640x480
        fx = 481.985
		fy = 481.985
		cx = 334.203
		cy = 241.948

         */

    private PIDEx turnPID;
    private PIDCoefficientsEx turnPIDCoeffs;
    private double GBfx = 481.985;
    private double GBfy = 481.985;
    private double GBcx = 334.203;
    private double GBcy = 241.948;
    public int exposure = 10;

    private double Kp = 1; // one of the main values, increase if it goes slowly or stops early and reduce slightly if it overshoots and oscillates
    private double Ki = 0.000; // keep 0 for aiming
    private double Kd = 0.0; // start with 0.1, if its overshooting increase a little bit (by 0.02-0.05), if it feels twitchy reduce it a little bit
    private double integralSumMax = 0.5;
    private double stabilityThreshold = 0.5;
    private double lowPassGain = 0.9;

    private double slowRotationScale = 0.35;


    double cameraBearing = 0;
    double botHeadingIMU;
    double botHeadingAtCapture;

    double desiredHeading;
    double currentHeading;

    boolean aiming;

    double turnError;
    double turnCommand = 0;

    private AprilTagDetection tag;

    private double power;


    @Override
    public void init() {

        initDriveAndIMU();

        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(DcMotor.class,"fly");
        feeder = hardwareMap.get(CRServo.class, "feeder");

        turnPIDCoeffs = new PIDCoefficientsEx(Kp, Ki, Kd, integralSumMax, stabilityThreshold, lowPassGain);
        turnPID = new PIDEx(turnPIDCoeffs);
        power = 0;


        fly = hardwareMap.get(DcMotorEx.class, "fly");
        fly.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(GBfx, GBfy, GBcx, GBcy) // need to input the values here after getting the intrinsics from the camera calibration
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .build();

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                telemetry.addData("Camera", "Waiting");
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure((long)exposure, TimeUnit.MILLISECONDS);
            telemetry.addData("Camera", "Ready & exposure set");
        }

        telemetry.addLine("Hardware Initialized!");

    }

    @Override
    public void loop() {

            // D-pad up/down to raise or lower target velocity
        if (gamepad1.dpadUpWasPressed())   targetVel += 50;   // +500 tps
        if (gamepad1.dpadDownWasPressed()) targetVel -= 50;   // -500 tps (reverse if negative)
        if (gamepad1.b)         targetVel = 0;

        targetVel = -gamepad1.left_stick_y*2750;
        fly.setVelocity(targetVel); // ticks per second (negative allowed)

        telemetry.addData("Target (tps)", targetVel);
        telemetry.addData("Actual (tps)", fly.getVelocity());

        //FEEDER
        if (Math.abs(gamepad2.left_stick_y) > 0.05) {
            feeder.setPower(gamepad2.left_stick_y);
        } else {
            feeder.setPower(0);
        }
        //Intake
        if (gamepad2.left_trigger > 0.05){
            intake.setPower( gamepad2.left_trigger);
        } else {
            intake.setPower(0);
        }

//OUTTAKE
        if(gamepad2.a){
            power = 0.98;
        } else if (gamepad2.b) {
            power = 0.90;
        } else if (gamepad2.y) {
            power = 0.8;
        } else if(gamepad2.x){
            power = 0.85;

        }else{
            power = 0;
        }

        outtake.setPower(power);

        botHeadingIMU = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        List<AprilTagDetection> detectionsList = tagProcessor.getDetections();

        if (detectionsList != null && !detectionsList.isEmpty()) {
            tag = detectionsList.get(0);

            if (tag.ftcPose != null) {
                bearing = AngleUnit.normalizeRadians(tag.ftcPose.bearing);

                telemetry.addData("exposure", exposure);
                telemetry.addData("AIM: turnCmd", "%.3f", turnCommand);
                telemetry.addData("Tag Info", String.format("ID: %d | Bearing: %.1f° | Dist: %.1f in", tag.id, Math.toDegrees(bearing), tag.ftcPose.range));
                // Offsets: X = left/right (positive right), Y = up/down (positive up), Z = forward/back (positive forward)
                // Range is straight-line distance from camera to tag in inches.
                telemetry.addData("Offsets (in)", String.format("X: %.1f  Y: %.1f  Z: %.1f", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
                telemetry.addData("tag solve time ms:", tagProcessor.getPerTagAvgPoseSolveTime());
                telemetry.addData("decisionMargin:", "%.1f", tag.decisionMargin);
                cameraBearing = AngleUnit.normalizeRadians(tag.ftcPose.bearing);
                telemetry.update();

            } else {
                telemetry.addLine("no pose");
            }
        } else {
            telemetry.addLine("AIM: no tag");
        }
        if (gamepad1.aWasPressed()){
            if (detectionsList != null && !detectionsList.isEmpty()){
                AprilTagDetection t = detectionsList.get(0);
                if (t.ftcPose != null){
                    aiming = true;
                    botHeadingAtCapture = botHeadingIMU;
                    desiredHeading = AngleUnit.normalizeRadians(botHeadingAtCapture + AngleUnit.normalizeRadians(t.ftcPose.bearing));
                }
            }


        }

        if (gamepad1.aWasReleased()){
            aiming = false;
        }

        if (gamepad1.a){
            aim();
        } else {
            handleDrivetrain();
        }
        telemetry.update();

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

        //telemetry.addData("heading", Math.toDegrees(heading));
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

}
