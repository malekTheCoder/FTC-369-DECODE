package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.UpdatedTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.RoadrunnerRobotLocalizer;

import java.util.function.Supplier;

@TeleOp(name = "regression making")
public class regressionMaking extends OpMode {
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;

    private Drivetrain drivetrain;
    private RoadrunnerRobotLocalizer robotLocalizer;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    private double blueGoalXPosition = 10;
    private double blueGoalYPosition = 125;

    private double distanceToGoal = 0;


    // Absolute angle from +X axis (field) toward the goal
    private double angleToGoalFieldRad = 0.0;

    // Angle the goal is relative to robot forward ( for turret)
    private double angleToGoalRelRobotRad = 0.0;
    private double angleToGoalRelRobotDeg = 0.0;
    private double multiplier = 1;//TEMP

    // forward is 90 degressd for pedro so this is correction
    private static final double PEDRO_FORWARD_OFFSET_RAD = Math.toRadians(0);

    private DcMotorEx turretMotor;
    private UpdatedTurret turret;

    private Servo stopper;

    private Outtake outtake;

    private Intake intake;

    private double targetVelocity;



    @Override
    public void init() {

        drivetrain = new Drivetrain(hardwareMap);
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turret = new UpdatedTurret(turretMotor);
        stopper = hardwareMap.get(Servo.class, "stopper");
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);

        robotLocalizer = new RoadrunnerRobotLocalizer(hardwareMap, new Pose2d(65,-65 , Math.toRadians(0)), RoadrunnerRobotLocalizer.AllianceColor.BLUE);

        // Driver Station / Driver Hub telemetry
        telemetry.addLine("Initialized and ready to start");
        telemetry.update();

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        robotLocalizer.updateBotPosition();
        if (gamepad2.leftBumperWasPressed()){
            targetVelocity -= 20;
        } else if (gamepad2.rightBumperWasPressed()) {
            targetVelocity += 20;
        }

        if(gamepad2.a){
            intake.runIntake(.85);
        }
        else{
            intake.stopIntake();
        }
        outtake.setTargetVelocity(targetVelocity);
        outtake.runOuttake();

//        turret.update(angleToGoalRelRobotDeg, telemetry);

//        if(gamepad2.xWasPressed()){
//            multiplier=0;
//        }
//
//        if(multiplier == 0){
//            turret.manual(gamepad2.left_stick_x);
//        }
//
//        if(gamepad2.left_trigger>.3){
//            turret.resetPosition();
//        }

        telemetry.addData("Bot Pose", robotLocalizer.getBotPosition());
        telemetry.addData("Distance to goal", robotLocalizer.getDistanceToGoal());
        telemetry.addData("target velocity", targetVelocity);
        telemetry.addData("actual velocity", outtake.getAverageVelocity());
        //turret.aim(0.9*multiplier);

//        telemetryM.update();
        telemetry.update();
    }


    private void resetBotPose(){
        follower = Constants.createFollower(hardwareMap);

        // follower.setStartingPose(new Pose(0,0,0));
        follower.setStartingPose(new Pose(144, 0, Math.toRadians(-180)));
        follower.update();
    }
}
