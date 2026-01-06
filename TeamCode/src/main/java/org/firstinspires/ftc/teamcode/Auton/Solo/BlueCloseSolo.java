package org.firstinspires.ftc.teamcode.Auton.Solo;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "BlueCloseSolo")
public class BlueCloseSolo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(-59,-42, Math.toRadians(233)); // initial pose from meep meep
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        /*Flywheel flywheel = new Flywheel(hardwareMap, telemetry);
        Kicker kicker = new Kicker(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Belt belt = new Belt(hardwareMap);
        Hood hood = new Hood(hardwareMap);*/



        TrajectoryActionBuilder goToShootPreload = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-8.5,-12),Math.toRadians(221));

        TrajectoryActionBuilder goToFirstBatchAndDriveInAndGoBackToShoot = goToShootPreload.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-8.5,-40), Math.toRadians(270)) // drive into first set of artifacts
                .strafeToLinearHeading(new Vector2d(-9,-12.5),Math.toRadians(221)); // go shoot first batch

        TrajectoryActionBuilder goLoopPathForSecondBatch = goToFirstBatchAndDriveInAndGoBackToShoot.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(13,-18), Math.toRadians(270)) // go to second set of artifacts
                .strafeToLinearHeading(new Vector2d(13,-43),Math.toRadians(270)) // drive in w spline
                .strafeToLinearHeading(new Vector2d(-9,-12.5),Math.toRadians(221)); // go shoot second batch

        TrajectoryActionBuilder goLoopPathForThirdBatch = goLoopPathForSecondBatch.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(36,-19), Math.toRadians(270)) // go to third set of artifacts
                .strafeToLinearHeading(new Vector2d(36, -42), Math.toRadians(270)) //drive into third row
                .strafeToLinearHeading(new Vector2d(-9,-12.5),Math.toRadians(221));
        // go shoot second batch

        TrajectoryActionBuilder goGetOffLaunchLine = goLoopPathForThirdBatch.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-9,-20),Math.toRadians(224)); // go shoot second batch


/*
        SequentialAction shootPreload = new SequentialAction(
                new ParallelAction(
                        goToShootPreload.build(),
                        flywheel.holdFlywheelVelocity(1810,2)
                ),
                new ParallelAction(
                        flywheel.holdFlywheelVelocity(1810,2.5),
                        intake.holdIntakePower(0.55,2.5),
                        belt.holdBeltPower(-0.4,2.5),
                        new SequentialAction(
                                new SleepAction(0.4),
                                kicker.kickerUp(),
                                kicker.kickerDown(),
                                new SleepAction(0.4),
                                kicker.kickerUp(),
                                kicker.kickerDown(),
                                new SleepAction(0.4),
                                kicker.kickerUp(),
                                kicker.kickerDown()
                        )

                )

        );*/


//        SequentialAction TestingshootPreload = new SequentialAction( // in testing
//                new RaceAction(
//                        flywheel.holdFlywheelVelocity(1650,3),
//                        goToShootPreload.build()
//                ),
//                new ParallelAction(
//                        flywheel.holdFlywheelVelocity(1650,3),
//                        intake.holdIntakePower(0.5,3),
//                        new SequentialAction(
//                                kicker.kickerUp(),
//                                kicker.kickerDown(),
//                                belt.holdBeltPower(-0.5,0.3),
//                                new RaceAction(
//                                        belt.holdBeltPower(-0.5,0.3),
//                                        kicker.kickerUp()
//                                ),
//                                belt.stopBelt(),
//                                kicker.kickerDown(),
//                                belt.holdBeltPower(-0.5,0.3),
//                                new RaceAction(
//                                        belt.holdBeltPower(-0.5,0.3),
//                                        kicker.kickerUp()
//                                )
//                        )
//
//                )
//        );

//        SequentialAction firstBatchLoop = new SequentialAction(
//                flywheel.stopFlywheel(0),
//                new ParallelAction(
//                        goToFirstBatchAndDriveInAndGoBackToShoot.build(),
//                        intake.holdIntakePower(0.6,1.5),
//                        belt.holdBeltPower(-0.8,1.5),
//                        new SequentialAction(
//                                new SleepAction(0.75),
//                                flywheel.holdFlywheelVelocity(1810,0.75)
//                        )
//                ),
//                new ParallelAction(
//                        flywheel.holdFlywheelVelocity(1810,2.5),
//                        intake.holdIntakePower(0.5,2.5),
//                        belt.holdBeltPower(-0.5,2.5),
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
//
//                )
//
//
//
//
//        );
//
//
//        SequentialAction SecondBatchLoop = new SequentialAction(
//                flywheel.stopFlywheel(0),
//                new ParallelAction(
//                        goLoopPathForSecondBatch.build(),
//                        intake.holdIntakePower(0.55,2.25),
//                        belt.holdBeltPower(-0.45,2.25),
//                        new SequentialAction(
//                                new SleepAction(1.75),
//                                flywheel.holdFlywheelVelocity(1810,.5)
//                        )
//                ),
//
//                new ParallelAction(
//                        flywheel.holdFlywheelVelocity(1810,2.5),
//                        intake.holdIntakePower(0.62,2.5),
//                        belt.holdBeltPower(-0.65,2.5),
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
//
//                )
//
//
//
//
//        );
//
//
//
//        SequentialAction ThirdBatchLoop = new SequentialAction(
//                flywheel.stopFlywheel(0),
//                new ParallelAction(
//                        goLoopPathForThirdBatch.build(),
//                        intake.holdIntakePower(0.6,3),
//                        belt.holdBeltPower(-0.8,3),
//                        new SequentialAction(
//                                new SleepAction(2),
//                                flywheel.holdFlywheelVelocity(1810,1)
//                        )
//                ),
//                new ParallelAction(
//                        flywheel.holdFlywheelVelocity(1810,2.5),
//                        intake.holdIntakePower(0.5,2.5),
//                        belt.holdBeltPower(-0.5,2.5),
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


        while (!opModeIsActive()){
            if (isStopRequested()){
                return;
            }

            telemetry.addData("Position during Init", initialPose);
            telemetry.update();
        }

//
//        Actions.runBlocking(
//                new SequentialAction(
//                        shootPreload,
//                        firstBatchLoop,
//                        SecondBatchLoop,
//                        ThirdBatchLoop,
//                        goGetOffLaunchLine.build(),
//                        kicker.kickerDown()
//
//                )
//
//        );
//
    }
}
