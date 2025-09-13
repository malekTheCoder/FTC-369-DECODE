package org.firstinspires.ftc.teamcode.visionBasic;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp
public class BasicLimelight extends OpMode {
    Limelight3A limelight;

    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    double pow = 0.1;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0); // Switch to pipeline number 0

        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        double tx = 0;
        double ty = 0;
        double ta = 0;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
             tx = result.getTx(); // How far left or right the target is (degrees)
             ty = result.getTy(); // How far up or down the target is (degrees)
             ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);

//            Pose3D botpose = result.getBotpose();
//            line to get the full pose, just doing x direction centering for this though
            }

        if (gamepad1.left_stick_x > 0.2) {
            frontRight.setPower(-2.5*pow);
            backRight.setPower(2.5*pow);
            frontLeft.setPower(2.5*pow);
            backLeft.setPower(-2.5*pow);
        } else if (gamepad1.left_stick_x < -0.2) {
            frontRight.setPower(2.5*pow);
            backRight.setPower(-2.5*pow);
            frontLeft.setPower(-2.5*pow);
            backLeft.setPower(2.5*pow);

        }
        else
        {
            if (Math.abs(tx) < 1){
                frontRight.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                backLeft.setPower(0);
            } else if (tx > 1) {
                frontRight.setPower(-pow);
                backRight.setPower(pow);
                frontLeft.setPower(pow);
                backLeft.setPower(-pow);

            } else if (tx < -1) {
                frontRight.setPower(pow);
                backRight.setPower(-pow);
                frontLeft.setPower(-pow);
                backLeft.setPower(pow);
            }

        }





    }
}
