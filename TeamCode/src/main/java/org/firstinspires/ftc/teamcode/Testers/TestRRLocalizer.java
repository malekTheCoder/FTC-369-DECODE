package org.firstinspires.ftc.teamcode.Testers;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.PoseHoldController;
import org.firstinspires.ftc.teamcode.Subsystems.RoadrunnerRobotLocalizer;
import org.firstinspires.ftc.teamcode.Subsystems.UpdatedTurret;


@TeleOp(name = "test rr localize")
public class TestRRLocalizer extends OpMode {
    RoadrunnerRobotLocalizer rrLocalizer;
    UpdatedTurret turret;

     Drivetrain drive;
    // Pose2d startPose = new Pose2d(72,-72, Math.toRadians(180));
    Pose2d startPose = new Pose2d(-10, -10, Math.toRadians(180));

    double lastHeadingDeg = 0;
    double fullHeading = 0;
    boolean firstLoop = true;

    private final PoseHoldController hold = new PoseHoldController();
    private final ElapsedTime loopTimer = new ElapsedTime();

    // FACTORY YAW SCALAR: 1.0031
    // NEW ATTEMPTED YAW SCALAR 1.0006

    @Override
    public void init() {
        rrLocalizer = new RoadrunnerRobotLocalizer(hardwareMap, startPose, RoadrunnerRobotLocalizer.AllianceColor.BLUE);
        drive = new Drivetrain(hardwareMap, 0);
        turret = new UpdatedTurret(hardwareMap.get(DcMotorEx.class, "turret"));
        loopTimer.reset();
    }

    @Override
    public void loop() {
        rrLocalizer.updateBotPosition();

        Pose2d currentPose = rrLocalizer.getBotPosition();
        double heading = Math.toDegrees(currentPose.heading.toDouble());

        double inverseHeading = rrLocalizer.getBotHeadingDegrees0To360();
        double yawScalar = rrLocalizer.getYawScalar();

        if (gamepad1.x) {
            drive.setPinpointHeadingAdjuster(-1 * inverseHeading);
            fullHeading = 0;
            firstLoop = true;
            lastHeadingDeg = heading;
        }

        // --- Pose hold (RR pose, PD -> robot-centric drive commands) ---
        double dt = loopTimer.seconds();
        loopTimer.reset();

        if (gamepad1.triangleWasPressed()) {
            hold.startHolding(currentPose);
        }

        if (gamepad1.triangle && hold.isHolding()) {
            PoseHoldController.DriveCommand cmd = hold.update(currentPose, dt);
            drive.driveRobotCentric(cmd.forward, cmd.strafe, cmd.turn);
        } else {
            // Normal driver control
            drive.handleDrivetrainWithPinpoint(gamepad1, inverseHeading);
        }

        if (gamepad1.triangleWasReleased()) {
            hold.stopHolding();
        }


        double angleForTurret = -rrLocalizer.getRobotAngleToGoalDegrees();
        //turret.update(angleForTurret, telemetry);
        //turret.aim(0.1);


        telemetry.addData("X", currentPose.position.x);
        telemetry.addData("Y", currentPose.position.y);
        telemetry.addData("full heading", fullHeading);
        telemetry.addData("inverse heading (degrees)", inverseHeading);
        telemetry.addData("yaw scalar", yawScalar);
        telemetry.addData("robot angle to goal degrees", rrLocalizer.getRobotAngleToGoalDegrees());

        if (hold.getTargetPose() != null) {
            telemetry.addData("hold target x", hold.getTargetPose().position.x);
            telemetry.addData("hold target y", hold.getTargetPose().position.y);
            telemetry.addData("hold target heading (deg)", Math.toDegrees(hold.getTargetPose().heading.toDouble()));
        }

        telemetry.update();
    }
}
