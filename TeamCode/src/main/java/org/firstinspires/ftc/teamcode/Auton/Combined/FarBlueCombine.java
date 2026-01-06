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

@Autonomous(name = "Far Blue Combined")
public class FarBlueCombine extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(61, -15, Math.PI);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
//        Flywheel flywheel = new Flywheel(hardwareMap, telemetry);
//        Kicker kicker = new Kicker(hardwareMap);
//        Intake intake = new Intake(hardwareMap);
//        Belt belt = new Belt(hardwareMap);
//        Hood hood = new Hood(hardwareMap);

        TrajectoryActionBuilder goToShootPreload = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(55,-19), Math.toRadians(202));


        TrajectoryActionBuilder goLoopForFirstSet = goToShootPreload.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(46,-23), Math.toRadians(270)) // go to first set of artifacts
                .strafeToLinearHeading(new Vector2d(46,-40), Math.toRadians(270)) // drive into first set of artifacts
                .strafeToLinearHeading(new Vector2d(54,-19), Math.toRadians(205)); // go back after grabbing first set of artifacts to shoot

        TrajectoryActionBuilder goLoopForGateBatch = goLoopForFirstSet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(31,-70), Math.toRadians(180)) // go to intake from gate
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(54,-19), Math.toRadians(202.5)); // go back to shoot

        TrajectoryActionBuilder goLoopForExtraRandom = goLoopForGateBatch.endTrajectory().endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(40,-48), Math.toRadians(0)) // go grab anywehre
                .strafeToLinearHeading(new Vector2d(62,-48), Math.toRadians(0))// go grab anywehre
                .strafeToLinearHeading(new Vector2d(54,-19), Math.toRadians(202.5)); // go to shoot

//
//        SequentialAction shootPreload = new SequentialAction(
//                new ParallelAction(
//                        goToShootPreload.build(),
//                        flywheel.holdFlywheelVelocity(2060, 0.75)
//                ),
//                new ParallelAction(
//                        flywheel.holdFlywheelVelocity(2060,1.2),
//                        intake.holdIntakePower(0.5,1.2),
//                        belt.holdBeltPower(-0.5,1.2),
//                        new SequentialAction(
//                                new SleepAction(0.4),
//                                kicker.kickerUp(),
//                                kicker.kickerDown(),
//                                new SleepAction(0.4),
//                                kicker.kickerUp(),
//                                kicker.kickerDown(),
//                                new SleepAction(0.4),
//                                kicker.kickerUp(),
//                                kicker.kickerDown()
//                        )
//                )
//        );
//
//        SequentialAction firstBatchLoop = new SequentialAction(
//                flywheel.stopFlywheel(0),
//                new ParallelAction(
//                        goLoopForFirstSet.build(),
//                        intake.holdIntakePower(0.6,2),
//                        belt.holdBeltPower(-0.8,2),
//                        new SequentialAction(
//                                new SleepAction(2.2),
//                                flywheel.holdFlywheelVelocity(2060,0.3)
//                        )
//                ),
//                new ParallelAction(
//                        flywheel.holdFlywheelVelocity(2060,1.2),
//                        intake.holdIntakePower(0.5,1.2),
//                        belt.holdBeltPower(-0.5,1.2),
//                        new SequentialAction(
//                                new SleepAction(0.4),
//                                kicker.kickerUp(),
//                                kicker.kickerDown(),
//                                new SleepAction(0.4),
//                                kicker.kickerUp(),
//                                kicker.kickerDown(),
//                                new SleepAction(0.4),
//                                kicker.kickerUp(),
//                                kicker.kickerDown()
//                        )
//                )
//
//
//
//
//        );
//
//        SequentialAction gateBatchLoop = new SequentialAction(
//                flywheel.stopFlywheel(0),
//                new ParallelAction(
//                        goLoopForGateBatch.build(),
//                        flywheel.stopFlywheel(0),
//                        intake.holdIntakePower(0.6,3.5),
//                        belt.holdBeltPower(-0.8,3.5),
//                        new SequentialAction(
//                                new SleepAction(5.5),
//                                flywheel.holdFlywheelVelocity(2060,0.2)
//                        )
//                ),
//                new ParallelAction(
//                        flywheel.holdFlywheelVelocity(2060,1.2),
//                        intake.holdIntakePower(0.5,1.2),
//                        belt.holdBeltPower(-0.4,1.2),
//                        new SequentialAction(
//                                new SleepAction(1),
//                                kicker.kickerUp(),
//                                kicker.kickerDown(),
//                                new SleepAction(0.35),
//                                kicker.kickerUp(),
//                                kicker.kickerDown(),
//                                new SleepAction(0.4),
//                                kicker.kickerUp(),
//                                kicker.kickerDown()
//                        )
//                )
//        );
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//        while (!opModeIsActive()){
//            telemetry.addData("Position during Init", initialPose);
//            telemetry.update();
//        }
//
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        goToShootPreload.build(),
//                        goLoopForFirstSet.build(),
//                        goLoopForGateBatch.build(),
//                        new ParallelAction(
//                                intake.holdIntakePower(0.6,8),
//                                belt.holdBeltPower(-0.6,8),
//                                goLoopForExtraRandom.build()
//
//                        )
//
////                        shootPreload,
////                        firstBatchLoop,
////                        gateBatchLoop,
////                        goLoopForExtraRandom.build(),
////                        kicker.kickerDown()
//                )
//
//
//        );

    }
}
