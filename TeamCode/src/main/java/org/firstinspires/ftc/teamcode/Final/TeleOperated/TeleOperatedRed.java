package org.firstinspires.ftc.teamcode.Final.TeleOperated;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Final.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.RoadrunnerRobotLocalizer;

@TeleOp(name = "★ TeleOperatedRed")

public class TeleOperatedRed extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, RoadrunnerRobotLocalizer.AllianceColor.RED, gamepad1, gamepad2, telemetry);
        robot.runOnInit();
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
