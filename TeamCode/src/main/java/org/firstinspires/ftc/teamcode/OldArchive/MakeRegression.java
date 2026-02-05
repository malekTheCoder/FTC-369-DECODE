package org.firstinspires.ftc.teamcode.OldArchive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.UpdatedTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;
@Disabled
@TeleOp(name = "MakeRegression")
public class MakeRegression extends OpMode {
    private FtcDashboard dashboard;
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;

    private Drivetrain drivetrain;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    private double blueGoalXPosition = 0;
    private double blueGoalYPosition = 144;

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

    public static boolean track = false;
    public static double targetVelocity;
    public static double currentVelocity;

    private Outtake outtake;

    private Intake intake;

    public static int baseline;//DELETE THIS LATEr


    @Override
    public void init() {

        drivetrain = new Drivetrain(hardwareMap);
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turret = new UpdatedTurret(turretMotor);
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);

        targetVelocity = 0;
        dashboard = FtcDashboard.getInstance();


        follower = Constants.createFollower(hardwareMap);

        // follower.setStartingPose(new Pose(0,0,0));
        follower.setStartingPose(startingPose == null ? new Pose(8, 8.5, Math.toRadians(90)) : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(58, 20))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(110), 0.8))
                .build();

//        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
//                .addPath(new Path(new BezierLine(follower::getPose, new Pose(58, 20))))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();

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
        telemetry.addData("distance", distanceToGoal);


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
            // drivetrain.handleDrivetrain(gamepad1);
        }
        if (gamepad1.yWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

//        if (gamepad1.a){
//            follower.holdPoint(follower.getPose());
//        } else {
//            follower.breakFollowing();
//        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.breakFollowing();
            automatedDrive = false;

        }
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("targetVelocity", outtake.getTargetVelocity());
        packet.put("currentVelocity", outtake.getCurrentVelocity());
        packet.put("line", baseline);
        dashboard.sendTelemetryPacket(packet);


        turret.update(angleToGoalRelRobotDeg);

        // turret.aim(1);


//        if (track){
//            turret.aim(.5);
//        }

        outtake.setTargetVelocity(targetVelocity);
        outtake.runOuttake();
        currentVelocity = outtake.getAverageVelocity();
        intake.runIntake(gamepad1.right_stick_y);




        telemetryM.update();
        telemetry.update();



    }
}
