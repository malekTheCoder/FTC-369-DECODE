package org.firstinspires.ftc.teamcode.teleop;
import android.util.Size;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.List;
import java.util.concurrent.TimeUnit;

public class DetectApriltagPlusAllign extends OpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    private ExposureControl exposure;
    private GainControl gain;
    private boolean exposureConfigured = false;
    private PIDEx turnPID;
    private PIDCoefficientsEx turnPIDCoeffs;

    private double targetBearing = 0;


    @Override
    public void init() {

        double Kp = 1.0;
        double Ki = 0;
        double Kd = 0.1;
        double integralSumMax = 0.5;
        double stabilityThreshold = 0.5;
        double lowPassGain = 0.9;

        turnPIDCoeffs = new PIDCoefficientsEx(Kp, Ki, Kd, integralSumMax, stabilityThreshold, lowPassGain);
        turnPID = new PIDEx(turnPIDCoeffs);

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
    }

    @Override
    public void init_loop() {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING && !exposureConfigured) {
            exposure = visionPortal.getCameraControl(ExposureControl.class);

            // if webcam supports exposure control we can minimize exposure to be able to get readings while moving
            // this will result in a darker image which can be counteracted by increasing the gain
            exposure.setMode(ExposureControl.Mode.Manual);
            exposure.setExposure(15, TimeUnit.MILLISECONDS);

            gain = visionPortal.getCameraControl(GainControl.class);
            gain.setGain(255);

            exposureConfigured = true;
        }
    }

    @Override
    public void loop() {

        List<AprilTagDetection> detections = tagProcessor.getFreshDetections();

        if (!detections.isEmpty()){
            AprilTagDetection tag = detections.get(0);

            double range = tag.ftcPose.range;
            double bearing = tag.ftcPose.bearing;

            double turnCommand = turnPID.calculate(targetBearing, bearing);
            if (turnCommand > 1) turnCommand = 1;
            if (turnCommand < -1) turnCommand = -1;
            
            double shooterCommand = shooterModel(range);



            telemetry.addLine(String.format("XYZ %.2f %.2f %.2f", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));

            telemetry.addData("exposure", exposure.isExposureSupported());

            telemetry.addData("tag id", tag.id);
            telemetry.addData("confidence: ", tag.decisionMargin);

//                telemetry.addData("roll", tag.ftcPose.roll);
//                telemetry.addData("pitch", tag.ftcPose.pitch);
//                telemetry.addData("yaw", tag.ftcPose.yaw);

        }

        telemetry.update();
    }

    private double shooterModel(double range) {
        return 0;
    }
}
