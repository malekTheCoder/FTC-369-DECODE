package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Outtake;


@TeleOp(name = "test outtake regression")
@Config
public class TestOuttake extends OpMode {
    Outtake outtake;

    private DcMotor intake;

    public static double target = 0; // ticks/sec target


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        outtake = new Outtake(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");

    }

    @Override
    public void loop() {
        intake.setPower(gamepad1.left_stick_y);

        outtake.setTargetVelocity(target);

        if(gamepad1.dpadUpWasPressed()){
            target+=25
        }

    }
}
