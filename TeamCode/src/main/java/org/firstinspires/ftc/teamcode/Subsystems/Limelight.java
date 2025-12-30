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

    Limelight(HardwareMap hardwareMap, int pipeline, IMU imu){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.imu = imu;
        limelight.start();
        limelight.pipelineSwitch(pipeline);
    }
    private void limelightOffset(){
        //Use law of cos with SAS to find the third side (d2)
        //Use law of sin to find the 2nd base angle of triangle
        //90-(2nd base angle) to find the error between fly heading and tag heading
        double B=Math.abs(90-tx);
        double x = distanceFromLLTOFly;
        double d = distanceFromLimelightToGoalInches;
        flyDistance = Math.sqrt(Math.pow(x, 2) + Math.pow(d, 2)-2*x*d*Math.cos(Math.toRadians(B))); // d2 is the distance to the april tag from the fly wheel, formula: sqrt(a^2+c^2-2ac cos(B)
        if(tx<0){
            flyTx = 90 - Math.toDegrees(Math.asin(d * (Math.sin(Math.toRadians(B))/ flyDistance))); //sin-1(d(Sin(B)/d2
            flyTx = flyTx*-1;
        }
        else{
            flyTx = 90 - Math.toDegrees(Math.asin(d * (Math.sin(Math.toRadians(B))/ flyDistance))); //sin-1(d(Sin(B)/d2
        }
    }
    public void updateLimelightInfo() {
        orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            ty = llResult.getTy();
            tx = llResult.getTx();
        }
//    else {
//        ty = 0;
//        tx = 0;
//    }

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

    public void poseToString(Telemetry telemetry){
        StringBuilder poseString = new StringBuilder();
        if(pose!= null) {
            for (int i = 0; i < pose.length; i++) {
                poseString.append(pose[i]);
            }
        }
        telemetry.addLine(poseString.toString());
    }
}
