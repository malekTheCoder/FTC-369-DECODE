package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "flywheel test velocity")
public class FlywheelPID extends OpMode {
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

    private double GBfx = 481.985;
    private double GBfy = 481.985;
    private double GBcx = 334.203;
    private double GBcy = 241.948;

    public int exposure = 10;


    @Override
    public void init() {
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


        telemetry.addLine("Hardware Initialized!");

    }

    @Override
    public void loop() {

            // D-pad up/down to raise or lower target velocity
        if (gamepad1.dpadUpWasPressed())   targetVel += 100;   // +500 tps
        if (gamepad1.dpadDownWasPressed()) targetVel -= 100;   // -500 tps (reverse if negative)
        if (gamepad1.b)         targetVel = 0;

        targetVel = -gamepad1.left_stick_y*2750;
        fly.setVelocity(targetVel); // ticks per second (negative allowed)

        telemetry.addData("Target (tps)", targetVel);
        telemetry.addData("Actual (tps)", fly.getVelocity());


        List<AprilTagDetection> detectionsList = tagProcessor.getDetections();

        if (detectionsList != null && !detectionsList.isEmpty()) {
            AprilTagDetection tag = detectionsList.get(0);

            if (tag.ftcPose != null){
            bearing = AngleUnit.normalizeRadians(tag.ftcPose.bearing);

            telemetry.addData("exposure", exposure);

            telemetry.addData("Tag Info", String.format("ID: %d | Bearing: %.1f° | Dist: %.1f in", tag.id, Math.toDegrees(bearing), tag.ftcPose.range));
            // Offsets: X = left/right (positive right), Y = up/down (positive up), Z = forward/back (positive forward)
            // Range is straight-line distance from camera to tag in inches.
            telemetry.addData("Offsets (in)", String.format("X: %.1f  Y: %.1f  Z: %.1f", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
            telemetry.addData("tag solve time ms:", tagProcessor.getPerTagAvgPoseSolveTime());
            telemetry.update();
        }

    } else {
        telemetry.addLine("AIM: no tag");
        telemetry.update();
    }

        telemetry.update();

    }
}
