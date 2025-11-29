package org.firstinspires.ftc.teamcode.teleop;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
@TeleOp(name = " TELEOP BLUE")
public class BlueTeleop extends OpMode {
    private FtcDashboard dashboard;

    private Limelight3A limelight;
    private DcMotorEx frontRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;

    private DcMotor intake;
    private DcMotorEx fly;
    private DcMotor belt;
    private Servo kicker;
    private IMU imu;

    private Servo rgbLight;

    private double distanceFromLLTOFly = 4.25;

    YawPitchRollAngles orientation;
    LLResult llResult;


    public static double targetVel = 0;
    public static double actualVel;

    private PIDEx turnPID;
    private PIDCoefficientsEx turnPIDCoeffs;

    public static double Kp = 1.55; // one of the main values, increase if it goes slowly or stops early and reduce slightly if it overshoots and oscillates
    public static double Ki = 0.0001; // keep 0 for aiming
    public static double Kd = 0.1; // start with 0.1, if its overshooting increase a little bit (by 0.02-0.05), if it feels twitchy reduce it a little bit
    public static double integralSumMax = 0.5;
    public static double stabilityThreshold = Math.toRadians(1);
    public static double  lowPassGain = 0.9;

    private double slowRotationScale = 0.35;

    double botHeadingIMU;
    double botHeadingAtCapture;

    double desiredHeading;
    double currentHeading;

    boolean aiming;

    double turnError;
    double turnCommand = 0;


    double targetOffsetAngle_Vertical;
    double limelightMountAngleDegrees = 15;
    double limelightLensHeightInches = 13;
    double goalHeightInches = 29.5;
    double beltPowerScale = 0.8;

    double angleToGoalDegrees;
    double angleToGoalRadians;
    double distanceFromLimelightToGoalInches;

    private double ty;
    private double tx;

    private double flyMultiplier;
    private double flyAddVelocity;
    private boolean manualFlywheelControl = false;
    private double intakeMultiplier;

    private boolean kickerUp;
    private double kickerUpPosition = 0.5;
    private double kickerDownPosition = 0;

    private double verticalTranslation;
    private double flyTx;
    private double d2; // change name


    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        initDriveAndIMU();
        initFlyWheel();

        intake = hardwareMap.get(DcMotor.class, "intake");
        kicker = hardwareMap.get(Servo.class, "kicker");
        belt = hardwareMap.get(DcMotor.class, "belt");
        rgbLight = hardwareMap.get(Servo.class, "rgb");

        kickerUp = false;

        turnPIDCoeffs = new PIDCoefficientsEx(Kp, Ki, Kd, integralSumMax, stabilityThreshold, lowPassGain);
        turnPID = new PIDEx(turnPIDCoeffs);

        intakeMultiplier = 0.6;
        verticalTranslation = 75;
        flyAddVelocity = 2110;

        dashboard = FtcDashboard.getInstance();
        limelight.start();
        limelight.pipelineSwitch(0);

        telemetry.addLine("Hardware Initialized!");
    }



    @Override
    public void loop() {
        updateLimelightInfo();
        handleFlywheel();
        handleIntake();
        handleBelt();
        handleKicker();
        handleRGB();
        limelightOffest();

        botHeadingIMU = AngleUnit.normalizeRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));


        if (gamepad1.aWasPressed()){
            // if valid detection
            if (llResult != null && llResult.isValid()){
                aiming = true;
                botHeadingAtCapture = botHeadingIMU;
                desiredHeading = AngleUnit.normalizeRadians(botHeadingAtCapture - AngleUnit.normalizeRadians(Math.toRadians(flyTx)));
                // minus 4 from the tx becasue the camera is to the left of the bot, centering the bot gets aroudn 4 tx
            }
        } else if (gamepad1.aWasReleased()){
            aiming = false;
        }


        if (gamepad1.a){
            aim();
        } else {
            handleDrivetrain();
        }


        telemetry.addData("Target (tps)", targetVel);
        telemetry.addData("Actual (tps)", fly.getVelocity());

        if (llResult != null && llResult.isValid()){
            Pose3D botPoseMT2 = llResult.getBotpose_MT2();
            Pose3D botPoseMT1 = llResult.getBotpose();


            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("Bot pose MT2", botPoseMT2.toString());
            telemetry.addData("Yaw MT2", botPoseMT2.getOrientation().getYaw());
            telemetry.addData("Bot pose MT1", botPoseMT1.toString());
            telemetry.addData("Yaw MT1", botPoseMT1.getOrientation().getYaw());
            telemetry.addData("Distance", distanceFromLimelightToGoalInches);
        }


        TelemetryPacket packet = new TelemetryPacket();
        packet.put("targetVel", targetVel);
        packet.put("actualVel", actualVel);  // or fly.getVelocity() if you prefer
        dashboard.sendTelemetryPacket(packet);

        telemetry.update();
    }

    private void limelightOffest(){
        //Use law of cos with SAS to find the third side (d2)
        //Use law of sin to find the 2nd base angle of triangle
        //90-(2nd base angle) to find the error between fly heading and tag heading
        double B=Math.abs(90-tx);
        double x = distanceFromLLTOFly;
        double d = distanceFromLimelightToGoalInches;
        d2 = Math.sqrt(Math.pow(x, 2) + Math.pow(d, 2)-2*x*d*Math.cos(Math.toRadians(B))); // d2 is the distance to the april tag from the fly wheel, formula: sqrt(a^2+c^2-2ac cos(B)
        if((90 - Math.toDegrees(Math.asin(d * (Math.sin(Math.toRadians(B))/d2))))+Math.toDegrees(Math.asin(d * (Math.sin(Math.toRadians(B))/d2)))>Math.toDegrees(Math.asin(d * (Math.sin(Math.toRadians(B))/d2)))){
            flyTx = 90 - Math.toDegrees(Math.asin(d * (Math.sin(Math.toRadians(B))/d2))); //sin-1(d(Sin(B)/d2
            flyTx = flyTx*-1;
        }
        else{
            flyTx = 90 - Math.toDegrees(Math.asin(d * (Math.sin(Math.toRadians(B))/d2))); //sin-1(d(Sin(B)/d2
        }
    }
    private void handleKicker(){
        if (gamepad2.leftBumperWasPressed()){
            if (kickerUp){
                kicker.setPosition(kickerDownPosition);
            } else if (!kickerUp){
                kicker.setPosition(kickerUpPosition);
            }

            if (kickerUp){
                kickerUp = false;
            } else if(!kickerUp){
                kickerUp = true;
            }


        }
    }

    private void handleFlywheel() {
        actualVel = fly.getVelocity();
        targetVel = shooterModel(distanceFromLimelightToGoalInches);

        if (gamepad2.aWasPressed()){
            if (flyMultiplier == 1){
                flyMultiplier = 0;
            }
            else{
                flyMultiplier = 1;
            }
        }
        if (gamepad2.bWasPressed()){
            if (!manualFlywheelControl){
                manualFlywheelControl = true;
            } else if (manualFlywheelControl){
                manualFlywheelControl = false;
            }
        }

        if (manualFlywheelControl){
            fly.setVelocity(flyAddVelocity);
        } else if (!manualFlywheelControl) {
            fly.setVelocity(targetVel * flyMultiplier); // ticks per second (negative allowed)

        }


}

//    private double shooterModel (double distanceInches){
//        return 10.80327*distanceInches+(1574.00755-verticalTranslation); // add regression here to return the velocity needed given the distance
//    }

private double shooterModel (double distanceInches){
    return 8.78571*distanceInches+1641.42857; // add regression here to return the velocity needed given the distance
}

private void handleRGB(){
    if((Math.abs(targetVel-actualVel)<80 && Math.abs(turnCommand)<.1)) {
        rgbLight.setPosition(0.5);
    }
    else{
        rgbLight.setPosition(.28);
    }
}

private void updateLimelightInfo() {
    orientation = imu.getRobotYawPitchRollAngles();
    limelight.updateRobotOrientation(orientation.getYaw());
    llResult = limelight.getLatestResult();
    if (llResult != null && llResult.isValid()) {
        ty = llResult.getTy();
        tx = llResult.getTx();
    }else {
        ty = 0;
        tx = 0;
    }

    targetOffsetAngle_Vertical = ty;
    angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
}


private void handleBelt() {
    belt.setPower(beltPowerScale * gamepad2.left_stick_y);
}

private void handleIntake(){
    intake.setPower(gamepad2.left_trigger * intakeMultiplier);
}
private void handleDrivetrain() {

    if (gamepad1.x){
        imu.resetYaw();
    }

    // drivetrain code to get inputs from controller and call the drive method w/ parameters
    double rightStickX = gamepad1.right_stick_x;
    if (Math.abs(rightStickX) < 0.05) rightStickX = 0;


    double x = gamepad1.left_stick_x;
    double y = -gamepad1.left_stick_y;
    double rx = (gamepad1.right_trigger - gamepad1.left_trigger + (rightStickX * slowRotationScale));

    drive(y, x, rx);
}

private void aim() {
    if(aiming) {
        currentHeading = botHeadingIMU;
        turnError = AngleUnit.normalizeRadians(desiredHeading - currentHeading);
        turnCommand = turnPID.calculate(0.0, turnError);

        if (turnCommand > 1) turnCommand = 1;
        if (turnCommand < -1) turnCommand = -1;

        drive(0, 0, turnCommand);
    }
}

public void drive(double forward, double strafe, double rotate){
    // Always field-centric: rotate the joystick vector by -heading
    double theta = Math.atan2(forward, strafe);
    double r = Math.hypot(strafe, forward);

    double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    double rotatedTheta = AngleUnit.normalizeRadians(theta - heading);

    double f = r * Math.sin(rotatedTheta);
    double s = r * Math.cos(rotatedTheta);

    // Mecanum kinematics
    double frontLeftPower = f + s + rotate;
    double backLeftPower = f - s + rotate;
    double frontRightPower = f - s - rotate;
    double backRightPower = f + s - rotate;

    // Normalize so no value exceeds 1.0
    double max = Math.max(1.0, Math.max(Math.abs(frontLeftPower),
            Math.max(Math.abs(backLeftPower),
                    Math.max(Math.abs(frontRightPower),
                            Math.abs(backRightPower)))));

    frontLeft.setPower(frontLeftPower / max);
    backLeft.setPower(backLeftPower / max);
    frontRight.setPower(frontRightPower / max);
    backRight.setPower(backRightPower / max);


}

private void initDriveAndIMU() {
    frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
    frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
    backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
    backRight = hardwareMap.get(DcMotorEx.class, "backRight");

    frontRight.setDirection(DcMotor.Direction.REVERSE);
    backRight.setDirection(DcMotor.Direction.REVERSE);

    frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    imu = hardwareMap.get(IMU.class, "imu");
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP));

    imu.initialize(parameters);

    imu.resetYaw();
    botHeadingIMU = imu.getRobotYawPitchRollAngles().getYaw();
}


private void initFlyWheel() {
    fly = hardwareMap.get(DcMotorEx.class, "fly");
    fly.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    fly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    fly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
}

}