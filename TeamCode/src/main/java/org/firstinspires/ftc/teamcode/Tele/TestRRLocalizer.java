package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;


@TeleOp(name = "test rr localize")
public class TestRRLocalizer extends OpMode {
    MecanumDrive mecanumDrive;
    Drivetrain drive;
//     Pose2d startPose = new Pose2d(72,-72, Math.toRadians(180));
   Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));

    double lastHeadingDeg = 0;
    double fullHeading = 0;
    boolean firstLoop = true;

    // FACTORY YAW SCALAR: 1.0031
    //NEW ATTEMPTED YAW SCALAR 1.0006



    //Pose2d startPose = new Pose2d(-59,-47, 234);

    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, startPose);
        drive = new Drivetrain(hardwareMap);

    }

    @Override
    public void loop() {
        mecanumDrive.updatePoseEstimate();

        Pose2d currentPose = mecanumDrive.localizer.getPose();
        double heading = Math.toDegrees(mecanumDrive.localizer.getPose().heading.toDouble());

        if (firstLoop) {
            lastHeadingDeg = heading;
            firstLoop = false;
        }
        double delta = heading - lastHeadingDeg;
        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;
        fullHeading += delta;
        lastHeadingDeg = heading;

        double inverseHeading = (360 + Math.toDegrees(mecanumDrive.localizer.getPose().heading.toDouble())) % 360;


        drive.handleDrivetrain(gamepad1);

        double yawScalar = mecanumDrive.localizer.printYawScalar();



        if (gamepad1.x){
            drive.resetIMU();
            fullHeading = 0;
            firstLoop = true;
            lastHeadingDeg = heading;
        }


        telemetry.addData("X", currentPose.position.x);
        telemetry.addData("Y", currentPose.position.y);
        telemetry.addData("full heading", fullHeading);
        telemetry.addData("inverse heading (degrees)", inverseHeading);
        telemetry.addData("yaw scalar", yawScalar);




        telemetry.update();




    }
}
