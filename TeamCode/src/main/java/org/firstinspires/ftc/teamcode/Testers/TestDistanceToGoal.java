package org.firstinspires.ftc.teamcode.Testers;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.RoadrunnerRobotLocalizer;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@Disabled
@TeleOp(name = "Test Distance To Goal", group = "Testers")
public class TestDistanceToGoal extends OpMode {

    private RoadrunnerRobotLocalizer rrLocalizer;
    private Pose2d startPose;
    private Drivetrain drive;

    @Override
    public void init() {
        startPose = PoseStorage.savedPose;
        rrLocalizer = new RoadrunnerRobotLocalizer(
                hardwareMap,
                startPose,
                RoadrunnerRobotLocalizer.AllianceColor.BLUE
        );
        drive = new Drivetrain(hardwareMap, 180);

        telemetry.addLine("Distance-to-goal tester ready.");
        telemetry.addData("start x", startPose.position.x);
        telemetry.addData("start y", startPose.position.y);
        telemetry.addData("start heading (deg)", Math.toDegrees(startPose.heading.toDouble()));
        telemetry.update();
    }

    @Override
    public void loop() {
        rrLocalizer.updateBotPosition();

        double inverseHeading = rrLocalizer.getBotHeadingDegrees0To360();
        drive.handleDrivetrainWithPinpoint(gamepad1, inverseHeading);

        Pose2d pose = rrLocalizer.getBotPosition();


        double distToGoal = rrLocalizer.getDistanceToGoal();
        double angleToGoalDeg = rrLocalizer.getRobotAngleToGoalDegrees();

        telemetry.addData("x", pose.position.x);
        telemetry.addData("y", pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
        telemetry.addLine("---------------");
        telemetry.addData("distance to goal (in)", distToGoal);
        telemetry.addData("angle to goal (deg)", angleToGoalDeg);
        telemetry.update();
    }
}
