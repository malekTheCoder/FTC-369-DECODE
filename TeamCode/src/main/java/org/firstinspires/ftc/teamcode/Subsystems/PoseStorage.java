package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseStorage {
    private PoseStorage() {}

//default
    // public static Pose2d savedPose = new Pose2d(72,-72, Math.toRadians(180));

     public static Pose2d savedPose = new Pose2d(0,0, Math.toRadians(0));

    public static double pinpointHeadingOffsetDriverRelative = 0;

}
