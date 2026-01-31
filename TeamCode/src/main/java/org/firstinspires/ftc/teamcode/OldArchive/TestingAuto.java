package org.firstinspires.ftc.teamcode.OldArchive;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous(name = "testing auto")
public class TestingAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private final Pose startPose = new Pose(28.000, 133.000, Math.toRadians(143)); // start pose of bot in auto
    private final Pose shootPreloadPose = new Pose(55, 83.320, Math.toRadians(180));
    private final Pose intakeFirstRowPose = new Pose(35, 83.876, Math.toRadians(180));
    private final Pose shootFirstRowPose = new Pose(59.753, 82.948, Math.toRadians(180));
    private final Pose controlPointForIntakingSecondRow = new Pose(44.907, 56.598);
    private final Pose endOfIntakingSecondRowPose = new Pose(35, 59.753, Math.toRadians(180));

    private Path pathToShootPreload;
    private PathChain grabAndGoToShootFirstRow;
    private PathChain goToIntakeSecondRow;

    public void buildPaths(){
        // first path initialized to go to shoot first
        pathToShootPreload = new Path(new BezierLine(startPose, shootPreloadPose));
        pathToShootPreload.setLinearHeadingInterpolation(startPose.getHeading(), shootPreloadPose.getHeading());


        grabAndGoToShootFirstRow = follower.pathBuilder()
                .addPath(new BezierLine(shootPreloadPose, intakeFirstRowPose))
                .setLinearHeadingInterpolation(shootPreloadPose.getHeading(), intakeFirstRowPose.getHeading())
                .addPath(new BezierLine(intakeFirstRowPose, shootFirstRowPose))
                .setLinearHeadingInterpolation(intakeFirstRowPose.getHeading(), shootFirstRowPose.getHeading())
                .build();

        Pose startCurve = shootFirstRowPose;
        Pose controlPoint = controlPointForIntakingSecondRow;
        Pose endCurve = endOfIntakingSecondRowPose;

        goToIntakeSecondRow = follower.pathBuilder()
                .addPath(new BezierCurve(startCurve, controlPoint, endCurve))
                .setLinearHeadingInterpolation(startCurve.getHeading(), endCurve.getHeading())
                .build();



    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // starting state, drive to shoot preload
                follower.followPath(pathToShootPreload);
                setPathState(1);
                break;


            case 1:
                // wait till preload path completes
                if (!follower.isBusy()){
                    follower.followPath(grabAndGoToShootFirstRow, true);
                    setPathState(2);
                }
                break;


            case 2:
                if (!follower.isBusy()){
                    follower.followPath(goToIntakeSecondRow, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()){
                    setPathState(-1);
                }
                break;

        }

    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }




    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(startPose);
        buildPaths();

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}


    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}


}
