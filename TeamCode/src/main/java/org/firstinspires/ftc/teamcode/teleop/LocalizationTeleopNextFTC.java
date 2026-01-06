package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "nextftc + pedro localization test")

public class LocalizationTeleopNextFTC extends NextFTCOpMode {
    private Follower follower;
    private Drivetrain drivetrain;
    private TelemetryManager telemetryM;
    public static Pose startingPose;

    @Override
    public void onInit(){
        drivetrain = new Drivetrain(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(startingPose == null ? new Pose(7.8,9,Math.toRadians(90)) : startingPose);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryM.addLine("Initialized");
        telemetryM.update();

        // Driver Station / Driver Hub telemetry
        telemetry.addLine("Initialized");
        telemetry.update();


    }

    @Override
    public void onUpdate(){
        if (gamepad1.x){
            drivetrain.resetIMU();
        }

        follower.update();

        Pose pose = follower.getPose();
        telemetryM.debug("pose", pose);
        telemetryM.debug("x", pose.getX());
        telemetryM.debug("y", pose.getY());
        telemetryM.debug("headingRad", pose.getHeading());
        telemetryM.debug("velocity", follower.getVelocity());

        // Mirror the same info to the Driver Station / Driver Hub
        telemetry.addData("pose", pose);
        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());
        telemetry.addData("headingRad", pose.getHeading());
        telemetry.addData("velocity", follower.getVelocity());

        drivetrain.handleDrivetrain(gamepad1);

        telemetryM.update();
        telemetry.update();
    }
}
