package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseStorage {
    private PoseStorage() {}


     public static Pose2d savedPose = new Pose2d(0,0, Math.toRadians(270));

    public static double pinpointHeadingOffsetDriverRelative = 0;

}
