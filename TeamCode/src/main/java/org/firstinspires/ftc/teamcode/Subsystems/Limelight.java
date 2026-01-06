package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Limelight {
    private Limelight3A limelight;
    private double flyTx;
    private double flyDistance;
    double targetOffsetAngle_Vertical;
    double limelightMountAngleDegrees = (18.6+15)/2;
    double limelightLensHeightInches = 16.857;
    double goalHeightInches = 29.5;
    double beltPowerScale = 1;

    LLResult llResult;
    double angleToGoalDegrees;
    double angleToGoalRadians;
    double distanceFromLimelightToGoalInches;
    private double distanceFromLLTOFly = 4.25;
    YawPitchRollAngles orientation;
    private double ty;
    private double tx;

    private IMU imu;

    private double[] pose;

    public Limelight(HardwareMap hardwareMap, int pipeline){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.imu = hardwareMap.get(IMU.class, "imu");
        limelight.start();
        limelight.pipelineSwitch(pipeline);
    }

    public void updateLimelightInfo() {
        orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            ty = llResult.getTy();
            tx = llResult.getTx();
        }

        targetOffsetAngle_Vertical = ty;
        angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
        distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }
    public double[] findRobotPos(double robotYaw){
        //double robotYaw = imu.getRobotYawPitchRollAngles().getYaw();
        limelight.updateRobotOrientation(robotYaw);
        if(llResult != null && llResult.isValid()){
            Pose3D botpose_mt2 = llResult.getBotpose_MT2();
            if(botpose_mt2 != null){
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                double heading = botpose_mt2.getOrientation().getYaw();
                pose = new double[]{x, y, heading};
                return pose;
            }
        }
        return null;
    }

    public void printTelem(double yaw, Telemetry telemetry){
        findRobotPos(yaw);
        StringBuilder poseString = new StringBuilder();
        if(pose!= null) {
            for (int i = 0; i < pose.length; i++) {
                poseString.append(pose[i]);
            }
        }
        telemetry.addLine(poseString.toString());
    }
}