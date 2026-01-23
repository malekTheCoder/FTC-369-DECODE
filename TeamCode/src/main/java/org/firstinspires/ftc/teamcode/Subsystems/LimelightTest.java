package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
public class LimelightTest extends OpMode {
    private Limelight limelight;

    @Override
    public void init() {
        limelight = new Limelight(hardwareMap, 0);
    }

    @Override
    public void loop() {
        limelight.updateLimelightInfo();
        telemetry.addData("Limelight megatag2 pos", limelight.findRobotPos(limelight.angleToGoalDegrees));
//        limelight.printTelem();
        telemetry.update();
    }
}
