package org.firstinspires.ftc.teamcode.Final;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.FinalTurret;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.PoseHoldController;
import org.firstinspires.ftc.teamcode.Subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.RoadrunnerRobotLocalizer;
import org.firstinspires.ftc.teamcode.Subsystems.Stopper;

public class Robot {
    private final Drivetrain drivetrain;
    private final RoadrunnerRobotLocalizer robotLocalizer;
    private final Intake intake;
    private final PoseHoldController poseHoldController;
    private final FinalTurret turret;
    private final Outtake outtake;
    private final Stopper stopper;


    public boolean holdPressed;
    public boolean holdReleased;
    public boolean holdHeld;

    public boolean shootPressed;
    public boolean shootReleased;
    public boolean shootHeld;

    public double intakeShootPower = 1; // feed power while shooting

    public double stopperDelay = 0.25;
    public double intakeDelayPower = 0;
    public double intakeDefaultPower = 0.85;
    private double intakeReversePower = -0.5;
    private long shootStartNs = 0;



    private boolean holdPoseActive = false;

    // Timing for pose hold seconds
    private long lastLoopTimeNs = System.nanoTime();

    private final Telemetry telemetry;

    private Gamepad gp1;
    private Gamepad gp2;

    RoadrunnerRobotLocalizer.AllianceColor allianceColor;


    public Robot(HardwareMap hardwareMap, RoadrunnerRobotLocalizer.AllianceColor allianceColor, Gamepad gp1, Gamepad gp2, Telemetry telemetry){
        drivetrain = new Drivetrain(hardwareMap, PoseStorage.pinpointHeadingOffsetDriverRelative);
        robotLocalizer = new RoadrunnerRobotLocalizer(hardwareMap, PoseStorage.savedPose, allianceColor);
        this.allianceColor = allianceColor;
        intake = new Intake(hardwareMap);
        poseHoldController = new PoseHoldController();

        turret = new FinalTurret(hardwareMap, allianceColor);


        outtake = new Outtake(hardwareMap);
        stopper = new Stopper(hardwareMap);
        this.telemetry = telemetry;
        this.gp1 = gp1;
        this.gp2 = gp2;

    }

    public void runOnInit(){
        stopper.engageStopper();
    }


    public void start() {
        turret.resetPosition();
        stopper.engageStopper();
        intake.runIntake(0.0);
        outtake.resetController();
        turret.setTurretMode(FinalTurret.Mode.LIMELIGHT_BASIC_MODE);

    }


    public void update() {

        // TIMER FOR POSE HOLD CONTROLLER
        long nowNs = System.nanoTime();
        double dt = (nowNs - lastLoopTimeNs) / 1e9;
        lastLoopTimeNs = nowNs;
        dt = Math.max(dt, 1e-6);



        // LOCALIZATION
        robotLocalizer.updateBotPosition();




        // TURRET CODE

        if (gp2.leftBumperWasPressed()){
            turret.setTurretMode(FinalTurret.Mode.LIMELIGHT_BASIC_MODE);
        }

        if (gp2.rightBumperWasPressed()){
            turret.setTurretMode(FinalTurret.Mode.LOCKED_TURRET_MODE);
        }

        if (gp2.xWasPressed()){
            turret.setTurretMode(FinalTurret.Mode.MANUAL_RESET_MODE);
        }
        if (gp2.x){
            turret.setManualResetHeld(true);
        }
        if (gp2.xWasReleased()){
            turret.setManualResetHeld(false);
        }

        turret.setBotErrorDeg(robotLocalizer.getAngleForTurretDegrees());
        turret.setManualControl(gp2.right_stick_x);
        turret.update();




        //OUTTAKE CODE
        outtake.setTargetVelocity(outtake.velocityRegressionModel(robotLocalizer.getDistanceToGoal()));
        outtake.runOuttake();



        // CONTROLLER INPUTS FOR SHOOTING
        holdPressed  = gp1.leftBumperWasPressed();
        holdReleased = gp1.leftBumperWasReleased();
        holdHeld     = gp1.left_bumper;

        shootPressed  = gp1.aWasPressed();
        shootReleased = gp1.aWasReleased();
        shootHeld     = gp1.a;

        //pose pull for pose hold controller
        Pose2d currentPose = robotLocalizer.getBotPosition();


        //POSE HOLD CODE

        if (holdPressed) {
            // Stop the bot before engaging pose-hold
            drivetrain.setZeroPower();

            poseHoldController.startHolding(currentPose);
            holdPoseActive = true;

            turret.setHoldTurretForPoseHold(true);
        }

        if (holdHeld && poseHoldController.isHolding()) {
            PoseHoldController.DriveCommand cmd = poseHoldController.update(currentPose, dt);
            drivetrain.driveRobotCentric(cmd.forward, cmd.strafe, cmd.turn);
        } else {
            // Normal driver control
            drivetrain.handleDrivetrainWithPinpoint(gp1, robotLocalizer.getBotHeadingDegrees0To360());
        }

        if (holdReleased) {
            poseHoldController.stopHolding();
            holdPoseActive = false;

            turret.setHoldTurretForPoseHold(false);
        }

        //SHOOTING COMMAND CODE

        if (shootPressed) {
            shootStartNs = nowNs; // start delay timer on rising edge
        }

        if (shootHeld) {
            stopper.disengageStopper();

            double elapsedSec = (nowNs - shootStartNs) / 1e9;

            double intakeCmd;
            if (elapsedSec < stopperDelay)
            {
                intakeCmd = intakeDelayPower;
            } else {
                intakeCmd = intakeShootPower;
            }

            intake.runIntake(intakeCmd);
            telemetry.addData("ShootDelaySec", elapsedSec);
            telemetry.addData("IntakeCmd", intakeCmd);

        } else {
            stopper.engageStopper();

            //default intake speed with reverse option
            if (gp2.y){
                intake.runIntake(intakeReversePower);
            } else if (gp2.left_bumper){
                intake.stopIntake();
            } else {
                intake.runIntake(intakeDefaultPower);
            }

            shootStartNs = 0;
        }

        // x adjuster - wont need for now
        if (gp2.dpadLeftWasPressed()){
            if (allianceColor == RoadrunnerRobotLocalizer.AllianceColor.RED){
                robotLocalizer.adjustRedGoalX(-2);
            } else if (allianceColor == RoadrunnerRobotLocalizer.AllianceColor.BLUE) {
                robotLocalizer.adjustBlueGoalX(-2);
            }
        }
        if (gp2.dpadRightWasPressed()){
            if (allianceColor == RoadrunnerRobotLocalizer.AllianceColor.RED){
                robotLocalizer.adjustRedGoalX(2);
            } else if (allianceColor == RoadrunnerRobotLocalizer.AllianceColor.BLUE) {
                robotLocalizer.adjustBlueGoalX(2);
            }
        }

        double distanceToGoalForReg = robotLocalizer.getDistanceToGoal();
        // regression adjuster
        if (gp2.dpadDownWasPressed()){
            if (distanceToGoalForReg < outtake.SWITCH_DISTANCE_VELOCITY_REGRESSION){
                outtake.adjustCloseRegression(-15);
            } else {
                outtake.adjustFarRegression(-15);
            }

        }
        else if (gp2.dpadUpWasPressed()){
            if (distanceToGoalForReg < outtake.SWITCH_DISTANCE_VELOCITY_REGRESSION){
                outtake.adjustCloseRegression(15);
            } else {
                outtake.adjustFarRegression(15);
            }
        }






        //bot position reset
        if (gp2.backWasPressed()){
            robotLocalizer.resetBotPoseInCorner();
        }

        if (gp1.startWasPressed()){
            drivetrain.setPinpointHeadingAdjuster(-robotLocalizer.getBotHeadingDegrees0To360());
        }




        telemetry.addData("x", robotLocalizer.getBotPosition().position.x);
        telemetry.addData("y", robotLocalizer.getBotPosition().position.y);
        telemetry.addData("heading", Math.toDegrees(robotLocalizer.getBotPosition().heading.toDouble()));
        telemetry.addData("distance to goal", robotLocalizer.getDistanceToGoal());



        turret.addTelemetry(telemetry);


        telemetry.update();
    }

}
