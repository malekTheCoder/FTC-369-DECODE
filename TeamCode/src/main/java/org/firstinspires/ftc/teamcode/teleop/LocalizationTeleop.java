package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.TurrLog;
import org.firstinspires.ftc.teamcode.Subsystems.TurretLogic;
import org.firstinspires.ftc.teamcode.Subsystems.UpdatedTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@TeleOp(name = "pedro only localization test")
public class LocalizationTeleop extends OpMode {
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Drivetrain drivetrain;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    private double blueGoalXPosition = 15;
    private double blueGoalYPosition = 130;

    private double distanceToGoal = 0;


    // Absolute angle from +X axis (field) toward the goal
    private double angleToGoalFieldRad = 0.0;

    // Angle the goal is relative to robot forward ( for turret)
    private double angleToGoalRelRobotRad = 0.0;
    private double angleToGoalRelRobotDeg = 0.0;

    // forward is 90 degressd for pedro so this is correction
    private static final double PEDRO_FORWARD_OFFSET_RAD = Math.toRadians(0);

    private DcMotorEx turretMotor;
    private UpdatedTurret turret;


    @Override
    public void init() {

        drivetrain = new Drivetrain(hardwareMap);
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turret = new UpdatedTurret(turretMotor);



        follower = Constants.createFollower(hardwareMap);

        // follower.setStartingPose(new Pose(0,0,0));
        follower.setStartingPose(startingPose == null ? new Pose(7.8, 9, Math.toRadians(90)) : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

//        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
//                .addPath(new Path(new BezierLine(follower::getPose, new Pose(58, 20))))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(110), 0.8))
//                .build();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(58, 20))))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        telemetryM.addLine("Initialized and ready to start");
        telemetryM.update();

        // Driver Station / Driver Hub telemetry
        telemetry.addLine("Initialized and ready to start");
        telemetry.update();

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        if (gamepad1.x) {
            drivetrain.resetIMU();
        }


        follower.update();

        Pose pose = follower.getPose();

        // Basic follower pose telemetry
        double poseX = pose.getX();
        double poseY = pose.getY();
        double poseHeadingDeg = Math.toDegrees(pose.getHeading());

        // Driver Station / Driver Hub
        telemetry.addData("Pose X", poseX);
        telemetry.addData("Pose Y", poseY);
        telemetry.addData("Heading (deg)", poseHeadingDeg);

        double xDiff = (blueGoalXPosition - pose.getX());
        double yDiff = (blueGoalYPosition - pose.getY());
        double term1 = xDiff * xDiff;
        double term2 = yDiff * yDiff;
        double sum = term1 + term2;
        distanceToGoal = Math.sqrt(sum);

        angleToGoalFieldRad = Math.atan2(yDiff, xDiff);

        // Convert robot heading bc forward in pedro is 90
        double headingPedroRad = pose.getHeading();
        double headingAdjustedRad = AngleUnit.normalizeRadians(headingPedroRad - PEDRO_FORWARD_OFFSET_RAD);

        //realtive to robot
        // postive -> goal is to the left
        // negative goal is to the right.
        angleToGoalRelRobotRad = AngleUnit.normalizeRadians(angleToGoalFieldRad - headingAdjustedRad);
        angleToGoalRelRobotDeg = Math.toDegrees(angleToGoalRelRobotRad);



        if (!automatedDrive) {
            drivetrain.handleDrivetrain(gamepad1);
        }
        if (gamepad1.yWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.breakFollowing();
            automatedDrive = false;

        }



            turret.update(angleToGoalRelRobotDeg, telemetry);

             //turret.aim(1);


        if (gamepad1.a){
            turret.aim(1);
        } else {
            turret.aim(0);
        }




            telemetryM.update();
            telemetry.update();



    }
}
