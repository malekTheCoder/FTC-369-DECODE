package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@TeleOp(name = "stopper test")
public class stopperTest extends OpMode {
    Servo stopper;
    Intake intake;

    @Override
    public void init() {
        stopper = hardwareMap.get(Servo.class, "stopper");
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.bWasPressed()){
            stopper.setPosition(.5);
        }

        if(gamepad1.dpadDownWasPressed()){
            stopper.setPosition(stopper.getPosition()+.05);
        }

        if(gamepad1.dpadUpWasPressed()){
            stopper.setPosition(stopper.getPosition()-.05);
        }

        intake.runIntake(-gamepad1.left_trigger);

        telemetry.addData("Stopper position", stopper.getPosition());
        telemetry.update();
    }
}
