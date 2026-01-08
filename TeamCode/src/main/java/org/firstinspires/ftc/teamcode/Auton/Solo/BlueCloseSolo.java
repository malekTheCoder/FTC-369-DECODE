package org.firstinspires.ftc.teamcode.Auton.Solo;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "BlueCloseSolo")
public class BlueCloseSolo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(-50, -49, Math.toRadians(234)); // initial pose from meep meep
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        /*Flywheel flywheel = new Flywheel(hardwareMap, telemetry);
        Kicker kicker = new Kicker(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Belt belt = new Belt(hardwareMap);
        Hood hood = new Hood(hardwareMap);*/



        TrajectoryActionBuilder goToShootPreload = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-9,-10), Math.toRadians(270)); // position to shoot zero batch

        TrajectoryActionBuilder goToFirstBatchAndDriveInAndGoBackToShoot = goToShootPreload.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-9,-20), Math.toRadians(270)) // go to first set of artifacts
                .strafeToLinearHeading(new Vector2d(-9,-45), Math.toRadians(270)) // drive into first set of artifacts
                .strafeToLinearHeading(new Vector2d(-9,-10), Math.toRadians(270)); // go back after grabbing first set of artifacts to shoot

        TrajectoryActionBuilder goLoopPathForSecondBatch = goToFirstBatchAndDriveInAndGoBackToShoot.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(15,-20), Math.toRadians(270)) // go to second set of artifacts
                .strafeToLinearHeading(new Vector2d(15,-45), Math.toRadians(270)) // drive into second set of artifacts
                .strafeToLinearHeading(new Vector2d(-9,-10), Math.toRadians(270)); // go back after grabbing second set of artifacts to shoot


        TrajectoryActionBuilder goLoopPathForThirdBatch = goLoopPathForSecondBatch.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(40,-20), Math.toRadians(270)) // go to third set of artifacts
                .strafeToLinearHeading(new Vector2d(40,-45), Math.toRadians(270)) // drive into third set of artifacts
                .strafeToLinearHeading(new Vector2d(-9,-10), Math.toRadians(270)); // go back after grabbing third set of artifacts to shoot


        TrajectoryActionBuilder goGetOffLaunchLine = goLoopPathForThirdBatch.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-9,-20),Math.toRadians(224)); // go shoot second batch
        while (!opModeIsActive()){
            if (isStopRequested()){
                return;
            }

            telemetry.addData("Position during Init", initialPose);
            telemetry.update();
        }


        Actions.runBlocking(
                new SequentialAction(
                        goToShootPreload.build(),
                        goToFirstBatchAndDriveInAndGoBackToShoot.build(),
                        goLoopPathForSecondBatch.build(),
                        goLoopPathForThirdBatch.build(),
                        goGetOffLaunchLine.build()
                        //kicker.kickerDown()

                )

        );

    }
}
