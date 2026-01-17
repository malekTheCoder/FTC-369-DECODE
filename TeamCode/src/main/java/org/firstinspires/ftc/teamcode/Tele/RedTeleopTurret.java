package org.firstinspires.ftc.teamcode.Tele;

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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.UpdatedTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@TeleOp(name = "TURRET RED TELEOP")
public class RedTeleopTurret extends OpMode {
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;

    private Drivetrain drivetrain;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    private double redGoalXPosition = 138;
    private double redGoalYPosition = 125;

    private double distanceToGoal = 0;


    // Absolute angle from +X axis (field) toward the goal
    private double angleToGoalFieldRad = 0.0;

    // Angle the goal is relative to robot forward ( for turret)
    private double angleToGoalRelRobotRad = 0.0;
    private double angleToGoalRelRobotDeg = 0.0;

    private double multiplier = 1;

    // forward is 90 degressd for pedro so this is correction
    private static final double PEDRO_FORWARD_OFFSET_RAD = Math.toRadians(0);

    private DcMotorEx turretMotor;
    private UpdatedTurret turret;

    private Servo stopper;

    private Outtake outtake;

    private Intake intake;


    @Override
    public void init() {

        drivetrain = new Drivetrain(hardwareMap);
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turret = new UpdatedTurret(turretMotor);
        stopper = hardwareMap.get(Servo.class, "stopper");
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);



        follower = Constants.createFollower(hardwareMap);

        // follower.setStartingPose(new Pose(0,0,0));
        follower.setStartingPose(startingPose == null ? new Pose(97, 124, Math.toRadians(0)) : startingPose); // close initial pose x 97 y 124, far initial pose = 97.5, 0
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

        if (gamepad1.a){
            stopper.setPosition(0.5);
            intake.runIntake(-0.8);
        } else {
            stopper.setPosition(0.6);

            if (gamepad1.b){
                intake.runIntake(-0.95);
            } else {
                intake.stopIntake();
            }

        }


        if(gamepad2.dpadUpWasPressed()){
            outtake.verticalTranslationClose +=25;
        }
        if(gamepad2.dpadDownWasPressed()){
            outtake.verticalTranslationClose -=25;
        }

        if(gamepad2.yWasPressed()){
            outtake.verticalTranslationFar +=25;
        }
        if(gamepad2.bWasPressed()){
            outtake.verticalTranslationFar -=25;
        }

        if (gamepad1.x) {
            drivetrain.resetIMU();
        }



        if (gamepad2.backWasPressed()){
            resetBotPose();
        }

        if (gamepad2.leftBumperWasPressed()){
            redGoalXPosition--;
        } else if (gamepad2.rightBumperWasPressed()) {
            redGoalXPosition++;
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
        telemetry.addData("goal x", redGoalXPosition);



        double xDiff = (redGoalXPosition - pose.getX());
        double yDiff = (redGoalYPosition - pose.getY());
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




        drivetrain.handleDrivetrain(gamepad1);

        if (gamepad1.yWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.breakFollowing();
            automatedDrive = false;

        }


        outtake.setTargetVelocity(outtake.velocityRegressionModel(distanceToGoal));
        outtake.runOuttake();

        turret.update(angleToGoalRelRobotDeg, telemetry);
        if(gamepad2.xWasPressed()){
            multiplier=0;
        }


        if(multiplier == 0){
            turret.manual(gamepad2.left_stick_x);
        }

        if(gamepad2.left_trigger>.3){
            turret.resetPosition();
        }

        turret.aim(0.9*multiplier);


        telemetryM.update();
        telemetry.update();



    }
    private void resetBotPose(){
        follower = Constants.createFollower(hardwareMap);

        // follower.setStartingPose(new Pose(0,0,0));
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));
        follower.update();
    }
}
