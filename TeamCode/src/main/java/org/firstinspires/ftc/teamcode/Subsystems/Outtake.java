package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake {
    DcMotorEx fly1;
    DcMotorEx fly2;

    private double currentVelocity;
    private double targetVelocity;

    private double fly1Multiplier = 1;
    private double fly2Multiplier = -1;

    private double kV = 0.00035; // main feedforward for velocity
    private double kS = 0.044; //feedforward just for static friction
    private double kP = 0.005; //proportional
    private double kD = 0; //derivative

    private double lastError;

    private long lastLoopTimeNs = 0;


    public Outtake(HardwareMap hardwareMap){
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class, "fly2");

        fly1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fly1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fly2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize timing so the first dt isn't huge
        lastLoopTimeNs = System.nanoTime();
    }

    public void runOuttake() {
        // dt in seconds (subsystem-safe; no OpMode needed)
        long nowNs = System.nanoTime();
        double dt = (nowNs - lastLoopTimeNs) / 1e9;
        lastLoopTimeNs = nowNs;
        dt = Math.max(dt, 1e-6);

        currentVelocity = getAverageVelocity();

        double error = targetVelocity - currentVelocity;
        double errorDerivative = (error - lastError) / dt;
        lastError = error;

        double feedforward = (kV * targetVelocity) + (kS * Math.signum(targetVelocity));
        double proportional = kP * error;
        double derivative = kD * errorDerivative;

        double powPID = feedforward + proportional + derivative;
        setPower(clamp(powPID, -1, 1));
    }

    private static double clamp(double x, double min, double max) {
        return Math.max(min, Math.min(max, x));
    }

    public double getTargetVelocity(){
        return targetVelocity;
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public void resetController() {
        lastError = 0.0;
        lastLoopTimeNs = System.nanoTime();
    }

    public double getAverageVelocity(){
        return (fly1.getVelocity()*fly1Multiplier + fly2.getVelocity()*fly2Multiplier)/2.0;
    }

    public double getFly1Velocity(){
        return fly1.getVelocity()*fly1Multiplier;
    }

    public double getFly2Velocity(){
        return fly2.getVelocity()*fly2Multiplier;
    }

    public double getCurrentVelocity(){
        return currentVelocity;
    }

    public double velocityRegressionModel(double distanceToGoal){
        return 3.47953*distanceToGoal+1473.50091 + 15;// 15
    }

    public void setPower(double power){
        fly1.setPower(power*fly1Multiplier);
        fly2.setPower(power*fly2Multiplier);
    }

    public void stopOuttake(){
        fly1.setPower(0);
        fly2.setPower(0);
    }

    public boolean atTargetVelocity(double threshold){
        return Math.abs(getAverageVelocity() - getTargetVelocity()) < threshold;
    }


    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Target Vel", targetVelocity);
        telemetry.addData("Curr Vel 1", getFly1Velocity());
        telemetry.addData("Cur Vel 2", getFly2Velocity());
        telemetry.addData("Avg Vel", getAverageVelocity());
        telemetry.addData("At target Vel", atTargetVelocity(20));

    }

    public void updatePIDValues(double kS, double kV, double kP, double kD) {
        this.kS = kS;
        this.kV = kV;
        this.kP = kP;
        this.kD = kD;
    }




}
