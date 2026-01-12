package org.firstinspires.ftc.teamcode.Auton.Solo;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

@Autonomous(name = "RedCloseSolo")
public class RedCloseSolo extends LinearOpMode {
    //TODO: work on pathing first anf finalize pathing for this red side, just run the pathing in actions.runblockign and figure that out before adding full auto with actions


    public class Turret{
        private double turretMinTicks = 0;
        private double turretMaxTicks = 853;
        private DcMotorEx turret;

        public Turret(HardwareMap hardwareMap){
            turret = hardwareMap.get(DcMotorEx.class, "turret");
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setTargetPosition(0);
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public class AimTurret implements Action {
            private double targetPosition;
            private double turretPow;

            public AimTurret(double targetPos, double turretPow){
                this.targetPosition = targetPos;
                this.turretPow = turretPow;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                turret.setTargetPosition((int)targetPosition);
                turret.setPower(turretPow);

                if (Math.abs(turret.getCurrentPosition() - targetPosition) < 10){
                    return false;
                } else {
                    return true;
                }

            }



        }

        public Action aimTurret(double targetPosition, double turretPower){
            return new Turret.AimTurret(targetPosition, turretPower );
        }


    }

    public class Intake{
        private DcMotor intakeMotor;

        public Intake (HardwareMap hardwareMap){
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }// constructor

        public class HoldIntakePower implements Action {
            private double power;
            private double duration;

            private ElapsedTime intakeTimer = new ElapsedTime();
            private boolean intakeTimerStarted = false;

            public HoldIntakePower(double power, double duration) {
                this.power = power;
                this.duration = duration;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!intakeTimerStarted){
                    intakeTimer.reset();
                    intakeTimerStarted = true;
                }
                intakeMotor.setPower(power);
                packet.put("intakePower", power);

                if(intakeTimer.seconds() > duration){
                    return false;
                } else {
                    return true;
                }
            }
        }

        public Action holdIntakePower(double power, double time) {
            return new Intake.HoldIntakePower(power, time);
        }

        // stop the intake
        public class StopIntake implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                intakeMotor.setPower(0);
                return false;
            }
        }

        public Action stopIntake() {
            return new Intake.StopIntake();
        }
    }

    public class Flywheel{
        Outtake outtake;

        public Flywheel(HardwareMap hardwareMap){
            outtake = new Outtake(hardwareMap);
        }

        public class RunFlywheel implements Action{
            private double targetVelocity;
            private double duration;

            private ElapsedTime flywheelTimer = new ElapsedTime();
            private boolean flywheelTimerStarted = false;

            public RunFlywheel(double targetVelocity, double seconds){
                this.targetVelocity = targetVelocity;
                this.duration = seconds;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (!flywheelTimerStarted){
                    flywheelTimer.reset();
                    flywheelTimerStarted = true;
                }

                outtake.setTargetVelocity(targetVelocity);
                outtake.runOuttake();

                if(flywheelTimer.seconds() > duration){
                    return false;
                } else {
                    return true;
                }

            }
        }

        public Action runFlywheel(double targetVelocity, double durationSeconds){
            return new Flywheel.RunFlywheel(targetVelocity, durationSeconds);
        }

        public class StopFlywheel implements Action{

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                outtake.stopOuttake();
                return false;
            }
        }

        public Action stopFlywheel(){
            return new Flywheel.StopFlywheel();
        }
    }

    public class Stopper{
        private Servo stopper;
        private double engagedPosition = 0.5; // fine tune this value
        private double disengagedPosition = 0.6; //fine tune this value
        private double servoTime = 0.25; // time it takes servo to move between postions

        public Stopper(HardwareMap hardwareMap){
            stopper = hardwareMap.get(Servo.class, "stopper");
        }

        public class EngageStopper implements Action{

            ElapsedTime stopperTimer;
            private boolean isStopperTimerReset = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!isStopperTimerReset){
                    stopperTimer = new ElapsedTime();
                    stopperTimer.reset();
                    isStopperTimerReset = true;
                }

                stopper.setPosition(engagedPosition);

                if (stopperTimer.seconds() > servoTime){
                    return false;
                } else {
                    return true;
                }

            }
        }

        public Action engageStopper(){
            return new Stopper.EngageStopper();
        }

        public class DisengageStopper implements Action{

            ElapsedTime stopperTimer;
            private boolean isStopperTimerReset = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!isStopperTimerReset){
                    stopperTimer = new ElapsedTime();
                    stopperTimer.reset();
                    isStopperTimerReset = true;
                }

                stopper.setPosition(disengagedPosition);

                if (stopperTimer.seconds() > servoTime){
                    return false;
                } else {
                    return true;
                }

            }
        }

        public Action disengageStopper(){
            return new Stopper.DisengageStopper();
        }



    }


    @Override
    public void runOpMode() throws InterruptedException {

          Pose2d initialPose = new Pose2d(-59,42, Math.toRadians(127)); // initial pose from meep meep
          MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);




        TrajectoryActionBuilder goToShootPreload = drive.actionBuilder(initialPose) //
                .strafeToLinearHeading(new Vector2d(-8.5, 8), Math.toRadians(138));

        TrajectoryActionBuilder goToFirstBatchAndDriveInAndGoBackToShoot = goToShootPreload.endTrajectory().fresh() //
                .strafeToLinearHeading(new Vector2d(-14, 52),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-14, 8), Math.toRadians(138));

        TrajectoryActionBuilder goLoopPathForSecondBatch = goToFirstBatchAndDriveInAndGoBackToShoot.endTrajectory().fresh() //
                .strafeToLinearHeading(new Vector2d(6, 26), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(6, 59), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-8.5, 8), Math.toRadians(138));

        TrajectoryActionBuilder goLoopPathForThirdBatch = goLoopPathForSecondBatch.endTrajectory().fresh() //
                .strafeToLinearHeading(new Vector2d(28, 24), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(28, 57), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-4, 8), Math.toRadians(138));

        TrajectoryActionBuilder goGetOffLaunchLine = goLoopPathForThirdBatch.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-9, 20), Math.toRadians(136));



        while (!opModeIsActive()){
            if (isStopRequested()){
                return;
            }

            telemetry.addData("Position during Init", initialPose);
            telemetry.update();
        }


        Actions.runBlocking(
                new SequentialAction(


                )

        );

    }
}
