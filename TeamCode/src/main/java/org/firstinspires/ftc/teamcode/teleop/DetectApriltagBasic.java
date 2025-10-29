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
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = " apriltag detect + info ")
public class DetectApriltagBasic extends OpMode {


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

    public int exposure = 6;


    @Override
    public void init() {

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


        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure((long)exposure, TimeUnit.MILLISECONDS);

        telemetry.addLine("Hardware Initialized!");


    }

    @Override
    public void loop() {


        handleAimBot();

    }

    private void handleAimBot() {


        List<AprilTagDetection> detectionsList = tagProcessor.getDetections();

        if (detectionsList != null && !detectionsList.isEmpty()) {
            AprilTagDetection tag = detectionsList.get(0);


            bearing = tag.ftcPose.bearing;

            telemetry.addData("exposure", exposure);

            telemetry.addData("AIM: id", tag.id);
            telemetry.addData("AIM: bearing (deg)", "%.1f", Math.toDegrees(bearing));
            telemetry.addData("tag solve time ms:", tagProcessor.getPerTagAvgPoseSolveTime());
            telemetry.update();
        } else {
            telemetry.addLine("AIM: no tag");
            telemetry.update();
        }


    }
}

