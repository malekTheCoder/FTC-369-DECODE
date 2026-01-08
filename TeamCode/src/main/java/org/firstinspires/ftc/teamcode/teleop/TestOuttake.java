package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Outtake;


@TeleOp(name = "test outtake")
@Config
public class TestOuttake extends OpMode {
    Outtake outtake;

    private DcMotor intake;

    public static double target = 2000; // ticks/sec target
    public static double KS = 0.044;
    public static double KV = 0.00035;
    public static double KP = 0.005;
    public static double KD = 0.0;
    public static double multiplier = 0.0;

    // Optional: threshold for "ready"
    public static double READY_THRESHOLD_TPS = 20;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        outtake = new Outtake(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");

    }

    @Override
    public void loop() {
        if (target <= 0) {
            outtake.stopOuttake();
            outtake.resetController();
            outtake.addTelemetry(telemetry);
            telemetry.update();
            return;
        }

        // Push dashboard values into the subsystem each loop
        outtake.updatePIDValues(KS*multiplier, KV*multiplier, KP*multiplier, KD*multiplier);
        outtake.setTargetVelocity(target);

        // Run the controller
        outtake.runOuttake();

        // Telemetry (graph these keys in Dashboard)
        outtake.addTelemetry(telemetry);
        telemetry.addData("Ready(threshold)", READY_THRESHOLD_TPS);
        telemetry.addData("Ready", outtake.atTargetVelocity(READY_THRESHOLD_TPS));
        telemetry.update();

        intake.setPower(gamepad1.left_stick_y);
    }
}
