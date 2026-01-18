package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierPoint;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous(name = "PathingTesting, just paths")
public class PathingTesting extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private final Pose startPose = new Pose(28.000, 133.000, Math.toRadians(143)); // start pose of bot in auto
    private final Pose shootPreloadPose = new Pose(55, 83.320, Math.toRadians(180));
    private final Pose intakeFirstRowPose = new Pose(25, 83.876, Math.toRadians(180));
    private final Pose shootFirstRowPose = new Pose(59.753, 82.948, Math.toRadians(180));
    private final Pose controlPointForIntakingSecondRow = new Pose(44.907, 56.598);
    private final Pose endOfIntakingSecondRowPose = new Pose(25, 65, Math.toRadians(180));
    private final Pose shootSecondRowPose = new Pose(60.124, 79.237, Math.toRadians(180));
    private final Pose controlPointForIntakingThirdRow = new Pose(51.031, 26.351);
    private final Pose endOfIntakingThirdRowPose = new Pose(23, 36.000);
    private final Pose shootThirdRowPose = new Pose(64.763, 78.309, Math.toRadians(180));

    private Path pathToShootPreload, pathToDriveIntoFirstRow, pathToGoToShootFirstRow, pathToIntakeSecondRow, pathToShootSecondRow, pathToIntakeThirdRow, pathToShootThirdRow;



    public void buildPaths(){
        // first path initialized to go to shoot first
        pathToShootPreload = new Path(new BezierLine(startPose, shootPreloadPose));
        pathToShootPreload.setLinearHeadingInterpolation(startPose.getHeading(), shootPreloadPose.getHeading());


        pathToDriveIntoFirstRow = new Path(new BezierLine(shootPreloadPose, intakeFirstRowPose));
        pathToDriveIntoFirstRow.setLinearHeadingInterpolation(shootPreloadPose.getHeading(), intakeFirstRowPose.getHeading());

        pathToGoToShootFirstRow = new Path(new BezierLine(intakeFirstRowPose, shootFirstRowPose));
        pathToGoToShootFirstRow.setLinearHeadingInterpolation(intakeFirstRowPose.getHeading(), shootFirstRowPose.getHeading());


        Pose startCurve = shootFirstRowPose;
        Pose controlPoint = controlPointForIntakingSecondRow;
        Pose endCurve = endOfIntakingSecondRowPose;

        pathToIntakeSecondRow = new Path(new BezierCurve(startCurve, controlPoint, endCurve));
        pathToIntakeSecondRow.setLinearHeadingInterpolation(startCurve.getHeading(), endCurve.getHeading());


        pathToShootSecondRow = new Path(new BezierLine(endOfIntakingSecondRowPose, shootSecondRowPose));
        pathToShootSecondRow.setLinearHeadingInterpolation(endOfIntakingSecondRowPose.getHeading(), shootSecondRowPose.getHeading());

        pathToIntakeThirdRow = new Path(new BezierCurve(shootSecondRowPose, controlPointForIntakingThirdRow, endOfIntakingThirdRowPose));
        pathToIntakeThirdRow.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180));

        pathToShootThirdRow = new Path(new BezierLine(endOfIntakingThirdRowPose, shootThirdRowPose));
        pathToShootThirdRow.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180));




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
                    follower.followPath(pathToDriveIntoFirstRow);
                    setPathState(2);
                }
                break;


            case 2:
                if (!follower.isBusy()){
                    follower.followPath(pathToGoToShootFirstRow);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()){
                    follower.followPath(pathToIntakeSecondRow);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()){
                    follower.followPath(pathToShootSecondRow);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()){
                    follower.followPath(pathToIntakeThirdRow);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()){
                    follower.followPath(pathToShootThirdRow);
                    setPathState(7);
                }

            case 7:
                if (!follower.isBusy()){
                    setPathState(-1);
                }
                break;

            default:
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
        setPathState(-1);

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
