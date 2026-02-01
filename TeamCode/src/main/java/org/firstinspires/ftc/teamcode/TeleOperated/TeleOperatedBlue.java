package org.firstinspires.ftc.teamcode.TeleOperated;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.RoadrunnerRobotLocalizer;


@TeleOp(name = "★ TeleOperatedBlue")
public class TeleOperatedBlue extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, RoadrunnerRobotLocalizer.AllianceColor.BLUE, gamepad1, gamepad2, telemetry);
    }

    @Override
    public void start(){
        robot.start();
    }

    @Override
    public void loop() {
        robot.update();
    }
}
