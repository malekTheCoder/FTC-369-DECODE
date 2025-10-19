package org.firstinspires.ftc.teamcode.teleop;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class DetectApriltagBaisc extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                //.setLensIntrinsics()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(640,480))
                .build();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING){}

        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);

        // if webcam supports exposure control we can minimize exposure to be able to get readings while moving
        // this will result in a darker image which can be counteracted by increasing the gain
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(15, TimeUnit.MILLISECONDS);

        GainControl gain = visionPortal.getCameraControl(GainControl.class);
        gain.setGain(255);


        waitForStart();

        while (!isStopRequested() && opModeIsActive()){

            tagProcessor.getFreshDetections();

            visionPortal.getCameraState();


            if (tagProcessor.getDetections().size() > 0){
                AprilTagDetection tag = tagProcessor.getDetections().get(0);



                telemetry.addLine(String.format("XYZ %.2f %.2f %.2f", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));

                telemetry.addData("exposure", exposure.isExposureSupported());

                telemetry.addData("tag id", tag.id);
                telemetry.addData("confidence: ", tag.decisionMargin);

                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);

            }

            telemetry.update();
        }
    }
}
