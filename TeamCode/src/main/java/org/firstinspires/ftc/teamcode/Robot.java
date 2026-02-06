package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.lynx.LynxModule;
import java.util.List;

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


    public boolean holdShootPressed;
    public boolean holdShootReleased;
    public boolean holdShootHeld;

    public boolean normalShootPressed;
    public boolean normalShootReleased;
    public boolean normalShootHeld;

    public double intakeShootPower = 0.4; // 0.85 - 0.90 recommended for feeding
    public double intakeDefaultSpeed = 0.8;


    private boolean holdShootActive = false;
    private boolean normalShootActive = false;

    // Timing for pose hold seconds
    private long lastLoopTimeNs = System.nanoTime();

    private final Telemetry telemetry;

    private Gamepad gp1;
    private Gamepad gp2;


    public Robot(HardwareMap hardwareMap, RoadrunnerRobotLocalizer.AllianceColor allianceColor, Gamepad gp1, Gamepad gp2, Telemetry telemetry){
        drivetrain = new Drivetrain(hardwareMap, PoseStorage.pinpointHeadingOffsetDriverRelative);
        robotLocalizer = new RoadrunnerRobotLocalizer(hardwareMap, PoseStorage.savedPose, allianceColor);
        intake = new Intake(hardwareMap);
        poseHoldController = new PoseHoldController();
        turret = new FinalTurret(hardwareMap);
        outtake = new Outtake(hardwareMap);
        stopper = new Stopper(hardwareMap);
        this.telemetry = telemetry;
        this.gp1 = gp1;
        this.gp2 = gp2;

    }


    public void start() {
        turret.resetPosition();
        stopper.engageStopper();
        intake.runIntake(0.0);
        outtake.resetController();
        turret.setMode(FinalTurret.Mode.AUTO);

    }


    public void update() {

        // dt (seconds) for pose-hold controller
        long nowNs = System.nanoTime();
        double dt = (nowNs - lastLoopTimeNs) / 1e9;
        lastLoopTimeNs = nowNs;
        dt = Math.max(dt, 1e-6);

        // localizer update
        robotLocalizer.updateBotPosition();

        // turret
        turret.setBotErrorDeg(robotLocalizer.getAngleForTurretDegrees());
        turret.update(telemetry);

        //flywheel
        outtake.setTargetVelocity(outtake.velocityRegressionModel(robotLocalizer.getDistanceToGoal()));
        outtake.runOuttake();

        // shooting inputs
        holdShootPressed  = gp1.triangleWasPressed();
        holdShootReleased = gp1.triangleWasReleased();
        holdShootHeld     = gp1.triangle;

        normalShootPressed  = gp1.crossWasPressed();
        normalShootReleased = gp1.crossWasReleased();
        normalShootHeld     = gp1.cross;

        // start hold shoot2
        if (holdShootPressed) {

            drivetrain.setZeroPower();
            poseHoldController.startHolding(robotLocalizer.getBotPosition());
            holdShootActive = true;

            normalShootActive = false;
        }

        // Start normal shoot
        if (!holdShootActive && normalShootPressed) {
            normalShootActive = true;
        }

        // stop shooting states on release
        if (holdShootReleased) {
            poseHoldController.stopHolding();
            holdShootActive = false;
        }
        if (normalShootReleased) {
            normalShootActive = false;
        }

        //drive
        if (holdShootHeld && holdShootActive && poseHoldController.isHolding()) {
            // pose hold drives robot-centric commands
            PoseHoldController.DriveCommand cmd = poseHoldController.update(robotLocalizer.getBotPosition(), dt);
            drivetrain.driveRobotCentric(cmd.forward, cmd.strafe, cmd.turn);

        } else {
            // normal drive
            drivetrain.handleDrivetrainWithPinpoint(gp1, robotLocalizer.getBotHeadingDegrees0To360());
        }

        // shoot on or no
        boolean shouldShoot = (holdShootHeld && holdShootActive) || (normalShootHeld && normalShootActive);

        if (shouldShoot) {
            stopper.disengageStopper();
            intake.runIntake(intakeShootPower);
        } else {
            stopper.engageStopper();
            intake.runIntake(intakeDefaultSpeed);
        }



        //bot position reset
        if (gp2.startWasPressed()){
            robotLocalizer.resetBotPoseInCorner();
        }

        if (gp1.squareWasPressed()){
            drivetrain.setPinpointHeadingAdjuster(-robotLocalizer.getBotHeadingDegrees0To360());
        }

        // telem
        telemetry.addData("TurretMode", turret.getMode());
        telemetry.addData("HoldShootActive", holdShootActive);
        telemetry.addData("NormalShootActive", normalShootActive);
        telemetry.addData("ShouldShoot", shouldShoot);
        telemetry.addData("OuttakeTargetVel", outtake.getTargetVelocity());
        telemetry.addData("OuttakeAvgVel", outtake.getAverageVelocity());
        telemetry.addData("Bot Pose", robotLocalizer.getBotPosition());
        telemetry.update();
    }

}
