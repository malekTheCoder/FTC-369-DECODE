package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class limelight extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    double ty;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        imu.resetYaw();

        limelight.start();
    }

    @Override
    public void loop() {

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        ty = llResult.getTy();
        double targetOffsetAngle_Vertical = ty;

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 0;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 14.5;

        // distance from the target to the floor
        double goalHeightInches = 29.5;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);


        if (llResult != null && llResult.isValid()){
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("Bot pose", botPose.toString());
            telemetry.addData("Yaw", botPose.getOrientation().getYaw());
            telemetry.addData("Distance", distanceFromLimelightToGoalInches);
        }


    }
}
