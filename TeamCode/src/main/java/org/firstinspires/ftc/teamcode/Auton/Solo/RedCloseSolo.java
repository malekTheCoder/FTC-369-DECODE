package org.firstinspires.ftc.teamcode.Auton.Solo;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "RedCloseSolo")
public class RedCloseSolo extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

          Pose2d initialPose = new Pose2d(-59,42, Math.toRadians(127)); // initial pose from meep meep
          MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
//        Flywheel flywheel = new Flywheel(hardwareMap, telemetry);
//        Kicker kicker = new Kicker(hardwareMap);
//        Intake intake = new Intake(hardwareMap);
//        Belt belt = new Belt(hardwareMap);
//        Hood hood = new Hood(hardwareMap);



        TrajectoryActionBuilder goToShootPreload = drive.actionBuilder(initialPose) //
                .strafeToLinearHeading(new Vector2d(-8.5, 8), Math.toRadians(138));

        TrajectoryActionBuilder goToFirstBatchAndDriveInAndGoBackToShoot = goToShootPreload.endTrajectory().fresh() //
                .strafeToLinearHeading(new Vector2d(-14, 52),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-14, 8), Math.toRadians(138));

        TrajectoryActionBuilder goLoopPathForSecondBatch = goToFirstBatchAndDriveInAndGoBackToShoot.endTrajectory().fresh() //
                .strafeToLinearHeading(new Vector2d(6, 26), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(6, 59), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-8.5, 8), Math.toRadians(138));

        TrajectoryActionBuilder goLoopPathForThirdBatch = goLoopPathForSecondBatch.endTrajectory().fresh() //
                .strafeToLinearHeading(new Vector2d(28, 24), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(28, 57), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-4, 8), Math.toRadians(138));

        TrajectoryActionBuilder goGetOffLaunchLine = goLoopPathForThirdBatch.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-9, 20), Math.toRadians(136));


//        SequentialAction shootPreload = new SequentialAction(
//                new ParallelAction(
//                        goToShootPreload.build(),
//                        flywheel.holdFlywheelVelocity(1810,2)
//                ),
//                new ParallelAction(
//                        flywheel.holdFlywheelVelocity(1810,2.5),
//                        intake.holdIntakePower(0.55,2.5),
//                        belt.holdBeltPower(-0.4,2.5),
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
//        );
//
//
//        SequentialAction firstBatchLoop = new SequentialAction(
//                flywheel.stopFlywheel(0),
//                new ParallelAction(
//                        goToFirstBatchAndDriveInAndGoBackToShoot.build(),
//                        intake.holdIntakePower(0.6,1.5),
//                        belt.holdBeltPower(-0.8,1.5),
//                        new SequentialAction(
//                                new SleepAction(0.7),
//                                flywheel.holdFlywheelVelocity(1810,0.8)
//                        )
//                ),
//                new ParallelAction(
//                        flywheel.holdFlywheelVelocity(1810,2.5),
//                        intake.holdIntakePower(0.5,2.5),
//                        belt.holdBeltPower(-0.5,2.5),
//                        new SequentialAction(
//                                new SleepAction(0.5),
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


//        Actions.runBlocking(
//                new SequentialAction(
//                        shootPreload,
//                        firstBatchLoop,
//                        SecondBatchLoop,
//                        ThirdBatchLoop,
//                        goGetOffLaunchLine.build(),
//                        kicker.kickerDown()
//
//
//
//                )
//
//        );

    }
}
