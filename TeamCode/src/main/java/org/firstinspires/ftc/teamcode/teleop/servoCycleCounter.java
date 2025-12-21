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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import android.os.Environment;

@TeleOp(name = "Servo counter")
public class servoCycleCounter extends OpMode {
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
    private double intakeMultiplier;

    private boolean kickerUp;
    private double kickerUpPosition = 0;
    private double kickerDownPosition = 1;

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

        turnPIDCoeffs = new PIDCoefficientsEx(Kp, Ki, Kd, integralSumMax, stabilityThreshold, lowPassGain);
        turnPID = new PIDEx(turnPIDCoeffs);

        intakeMultiplier = 0.6;
        verticalTranslation = 75;

        dashboard = FtcDashboard.getInstance();
        limelight.start();

        telemetry.addLine("Hardware Initialized!");
    }



    @Override
    public void loop() {
        updateLimelightInfo();

        if(gamepad1.xWasPressed()){
            writeToFile(Environment.getExternalStorageDirectory().getPath()+"/ServoCycleCount.txt",1, "servoName: ");
            //telemetry.addData("filepath/name", Environment.getExternalStorageDirectory().getPath()+"/ServoCycleCount.txt");
            telemetry.addLine("printed to file");
        }
        if(gamepad1.bWasPressed()){
            writeToFile(Environment.getExternalStorageDirectory().getPath()+"/ServoCycleCount.txt",-1, "servoName: ");
        }
        telemetry.update();
    }




    public void writeToFile(String filename, int data, String label) {
        File file = new File(Environment.getExternalStorageDirectory()+"/ServoCycleCount.txt");
        String[] readData = null;
        try (BufferedReader reader = new BufferedReader(new FileReader(filename))) {
            String line;
            int readDataInt;
            // Read each line until the end of the file (returns null)
            while ((line = reader.readLine()) != null) {
                readData = line.split(" ");
                readDataInt = Integer.parseInt(readData[1]);
                readData[1]= String.valueOf(readDataInt+=data);
                telemetry.addData("readData", readData[1]);
                telemetry.addLine(line);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }


        //PRINT to file
        try (PrintWriter out = new PrintWriter(new FileWriter(filename))) {
            if(readData!=null) {
                //out.println("ServoCycleCounter");
                out.println(label + readData[1]);
            }
            if(readData == null){
                out.println(label + "0");
            }

            if (file.createNewFile()) {
                telemetry.addLine("File created successfully: " + file.getAbsolutePath());
            } else {
                telemetry.addLine("File already exists: " + file.getAbsolutePath());
            }
        } catch (IOException e) {
            telemetry.addLine("Error writing to file: " + e.getMessage());
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


