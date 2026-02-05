package org.firstinspires.ftc.teamcode.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.RoadrunnerRobotLocalizer;
import org.firstinspires.ftc.teamcode.Subsystems.UpdatedTurret;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@Config
@TeleOp(name = "Turret KV Goal Tuner", group = "Tuning")
public class TurretKVTuner extends OpMode {

    // ===== Dashboard tunables =====
    public static double KP = 0.011;
    public static double KI = 0.0;
    public static double KD = 0.0004;

    public static double KS = 0.09;
    public static double KV = 0.0015;   // <-- tune this live

    public static double MAX_OUTPUT = 1.0;
    public static boolean ENABLE_TURRET = true;
    // ==============================

    private RoadrunnerRobotLocalizer rr;
    private UpdatedTurret turret;
    private Drivetrain drive;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rr = new RoadrunnerRobotLocalizer(
                hardwareMap,
                PoseStorage.savedPose,
                RoadrunnerRobotLocalizer.AllianceColor.BLUE
        );

        turret = new UpdatedTurret(hardwareMap.get(DcMotorEx.class, "turret"));
        drive = new Drivetrain(hardwareMap, 180);

        telemetry.addLine("Turret KV Goal Tuner ready.");
        telemetry.addLine("Dashboard -> Config -> TurretKVGoalTuner");
        telemetry.update();
    }

    @Override
    public void loop() {
        rr.updateBotPosition();

        // Drive the robot so the turret target changes while moving
        double inverseHeading = rr.getBotHeadingDegrees0To360();
        drive.handleDrivetrainWithPinpoint(gamepad1, inverseHeading);

        // This should be your goal-tracking angle output (the one you already use)
        double botErrorDeg = rr.getAngleForTurretDegrees();

        // Update turret target + targetVel inside UpdatedTurret
        turret.update(botErrorDeg);

        // Push tuned gains into the turret PID
        turret.getPid().setCoefficients(KP, KI, KD, KS, KV, MAX_OUTPUT);

        if (ENABLE_TURRET) {
            turret.aimPIDF();
        }

        // ====== Graphing signals (FTC Dashboard Graph) ======
        double targetTicks = turret.getTargetTicks();
        double currentTicks = turret.getCurrentTicks();
        double errorTicks = targetTicks - currentTicks;

        telemetry.addData("graph_targetTicks", targetTicks);
        telemetry.addData("graph_currentTicks", currentTicks);
        telemetry.addData("graph_errorTicks", errorTicks);
        telemetry.addData("graph_targetVel", turret.getTargetVelocityTicksPerSec());
        telemetry.update();
    }
}