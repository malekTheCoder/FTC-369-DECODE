package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class RoadrunnerRobotLocalizer {
    MecanumDrive drive;
    private double blueGoalX = 135;
    private double blueGoalY = -10;
    private double redGoalX = -67;
    private double redGoalY = 67;

    public double GOAL_X;
    public double GOAL_Y;

    public enum AllianceColor {BLUE, RED}
    AllianceColor allianceColor;


    public RoadrunnerRobotLocalizer(HardwareMap hardwareMap, Pose2d startPose, AllianceColor allianceColor){
        drive = new MecanumDrive(hardwareMap, startPose);
        this.allianceColor = allianceColor;

        if (allianceColor == AllianceColor.BLUE){
            GOAL_X = blueGoalX;
            GOAL_Y = blueGoalY;
        }
        else if (allianceColor == AllianceColor.RED) {
            GOAL_X = redGoalX;
            GOAL_Y = redGoalY;
        }
    }

    public void updateBotPosition(){
        drive.updatePoseEstimate();
    }

    public Pose2d getBotPosition(){
        return drive.localizer.getPose();
    }

    public double getBotHeadingDegrees() {
        return Math.toDegrees(getBotPosition().heading.toDouble());
    }

    public double getBotHeadingDegrees0To360() {
        double deg = Math.toDegrees(getBotPosition().heading.toDouble());
        return (deg % 360.0 + 360.0) % 360.0;
    }

    public void setBotPosition(Pose2d pose){
        drive.localizer.setPose(pose);
    }

    public double getDistanceToGoal(){
        Pose2d currentPose = getBotPosition();
        return Math.hypot((GOAL_X - currentPose.position.x), (GOAL_Y - currentPose.position.y));
    }

    public double getAngleToGoalRadians(){
        Pose2d currentPose = getBotPosition();
        return Math.atan2((GOAL_Y - currentPose.position.y), (GOAL_X - currentPose.position.x));
    }

    public double getRobotAngleToGoalRadians() {
        Pose2d currentPose = getBotPosition();

        // field angle - robot heading --> normalize and return
        return AngleUnit.normalizeRadians(getAngleToGoalRadians() - currentPose.heading.toDouble());
    }

    public double getRobotAngleToGoalDegrees(){
        return Math.toDegrees(getRobotAngleToGoalRadians());
    }

    public double getYawScalar(){
        return drive.localizer.printYawScalar();
    }

}
