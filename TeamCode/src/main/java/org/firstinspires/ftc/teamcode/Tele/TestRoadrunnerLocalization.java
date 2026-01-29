package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PinpointLocalizer;

public class TestRoadrunnerLocalization extends OpMode {

    PinpointLocalizer localizer;
    Pose2d startPose = new Pose2d(0,0,0);
    private double inchesPerTick = 0;

    @Override
    public void init() {

        localizer = new PinpointLocalizer(hardwareMap, inchesPerTick, startPose);

    }

    @Override
    public void loop() {

    }
}
