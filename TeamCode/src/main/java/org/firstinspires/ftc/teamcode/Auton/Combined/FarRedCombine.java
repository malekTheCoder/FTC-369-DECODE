package org.firstinspires.ftc.teamcode.Auton.Combined;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Far Red Combined")
public class FarRedCombine extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(61, 10, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
//        Flywheel flywheel = new Flywheel(hardwareMap, telemetry);
//        Kicker kicker = new Kicker(hardwareMap);
//        Intake intake = new Intake(hardwareMap);
//        Belt belt = new Belt(hardwareMap);
//        Hood hood = new Hood(hardwareMap);

        TrajectoryActionBuilder goToShootPreload = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(53,12), Math.toRadians(90));


        TrajectoryActionBuilder goLoopForFirstSet = goToShootPreload.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(32,30), Math.toRadians(90)) // go to first set of artifacts
                .strafeToLinearHeading(new Vector2d(32,50), Math.toRadians(90)) // drive into first set of artifacts
                .strafeToLinearHeading(new Vector2d(55,15), Math.toRadians(90)); // go back after grabbing first set of artifacts to shoot

        TrajectoryActionBuilder goLoopForSecondSet = goLoopForFirstSet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(10,30), Math.toRadians(90)) // go to second set of artifacts
                .strafeToLinearHeading(new Vector2d(10,50), Math.toRadians(90)) // drive into second set of artifacts
                .strafeToLinearHeading(new Vector2d(55,15), Math.toRadians(90)); // go back after grabbing second set of artifacts to shoot

        TrajectoryActionBuilder goLoopForWallSet = goLoopForSecondSet.endTrajectory().endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(46,79), Math.toRadians(10)) // wall set
                .strafeToLinearHeading(new Vector2d(67,79), Math.toRadians(10)) // drive in
                .strafeToLinearHeading(new Vector2d(48,15), Math.toRadians(90)); // go back after grabbing wall set

        Actions.runBlocking(
                new SequentialAction(
                        goToShootPreload.build(),
                        goLoopForFirstSet.build(),
                        goLoopForSecondSet.build(),
                        goLoopForWallSet.build()
//                        kicker.kickerDown()
                )



        );
    }
}
