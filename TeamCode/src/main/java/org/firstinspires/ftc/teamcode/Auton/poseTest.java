package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auton.Solo.BlueCloseSolo;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "pose test")
public class poseTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-62, -37.5, Math.toRadians(270)); // initial pose from meep meep
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
//        BlueCloseSolo.Turret turret = new BlueCloseSolo.Turret(hardwareMap);
//        BlueCloseSolo.Flywheel flywheel = new BlueCloseSolo.Flywheel(hardwareMap);
//        BlueCloseSolo.Intake intake = new BlueCloseSolo.Intake(hardwareMap);
//        BlueCloseSolo.Stopper stopper = new BlueCloseSolo.Stopper(hardwareMap);




        TrajectoryActionBuilder goToShootPreload = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-11,-18), Math.toRadians(270)); // position to shoot zero batch

//        TrajectoryActionBuilder goToFirstSet = goToShootPreload.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(-9,-20), Math.toRadians(270)); // go to first set of artifacts

        TrajectoryActionBuilder driveIntoFirstSet = goToShootPreload.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-11,-53), Math.toRadians(270)); // drive into first set of artifacts

        TrajectoryActionBuilder goEmptyGate = driveIntoFirstSet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-2,-55), Math.toRadians(165)); // drive into first set of artifacts

        TrajectoryActionBuilder goToShootFirstSet = goEmptyGate.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-11,-18), Math.toRadians(270)); // go back after grabbing first set of artifacts to shoot

        TrajectoryActionBuilder goToSecondSet = goToShootFirstSet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(13,-28), Math.toRadians(270)); // go to second set of artifacts

        TrajectoryActionBuilder driveIntoSecondSet = goToSecondSet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(13,-53), Math.toRadians(270)); // drive into second set of artifacts

        TrajectoryActionBuilder goToShootSecondSet = driveIntoSecondSet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-2,-15), Math.toRadians(270)); // go back after grabbing second set of artifacts to shoot

        TrajectoryActionBuilder goToThirdSet = goToShootSecondSet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(38,-28), Math.toRadians(270)); // go to third set of artifacts

        TrajectoryActionBuilder driveIntoThirdSet = goToThirdSet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(38,-56), Math.toRadians(270)); // drive into third set of artifacts

        TrajectoryActionBuilder goToShootThirdSet = driveIntoThirdSet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-2,-15), Math.toRadians(270)); // go back after grabbing third set of artifacts to shoot

        TrajectoryActionBuilder goGetOffLaunchLine = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(0,0),Math.toRadians(270)); // go shoot second batch



        while (!opModeIsActive()){
            if (isStopRequested()){
                return;
            }

            telemetry.addData("Position during Init", initialPose);
            telemetry.update();
        }

        Actions.runBlocking(
                new SequentialAction(
                    goGetOffLaunchLine.build()
                )

        );

        Pose2d endPose = drive.localizer.getPose();
        telemetry.addData("END POSE", endPose);
        telemetry.update();
        sleep(2000);

    }


}