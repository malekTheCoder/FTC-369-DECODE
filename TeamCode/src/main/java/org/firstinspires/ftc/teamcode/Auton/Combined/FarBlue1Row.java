package org.firstinspires.ftc.teamcode.Auton.Combined;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
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

@Autonomous(name = "Far Blue Auto")
public class FarBlue1Row extends LinearOpMode {

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

        public class AimTurret implements Action{
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
            return new AimTurret(targetPosition, turretPower );
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
            return new HoldIntakePower(power, time);
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
            return new StopIntake();
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
            return new RunFlywheel(targetVelocity, durationSeconds);
        }

        public class StopFlywheel implements Action{

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                outtake.stopOuttake();
                return false;
            }
        }

        public Action stopFlywheel(){
            return new StopFlywheel();
        }
    }

    public class Stopper{
        private Servo stopper;
        private double engagedPosition = 0.6; // fine tune this value
        private double disengagedPosition = 0.5; //fine tune this value
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
            return new EngageStopper();
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
            return new DisengageStopper();
        }



    }


    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(64, -10, Math.toRadians(270));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Turret turret = new Turret(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Stopper stopper = new Stopper(hardwareMap);



        TrajectoryActionBuilder goToShootPreload = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(53,-12), Math.toRadians(270));

        TrajectoryActionBuilder goToFirstSet = goToShootPreload.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(35,-30), Math.toRadians(270)); // go to first set of artifacts

        TrajectoryActionBuilder driveIntoFirstSet = goToFirstSet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(35,-53), Math.toRadians(270)); // drive into first set of artifacts

        TrajectoryActionBuilder goToShootFirstSet = driveIntoFirstSet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(58,-15), Math.toRadians(270)); // go back after grabbing first set of artifacts to shoot
//
//        TrajectoryActionBuilder goToSecondSet = goToShootFirstSet.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(7,-30), Math.toRadians(270)); // go to second set of artifacts
//        TrajectoryActionBuilder driveIntoSecondSet = goToSecondSet.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(7,-53), Math.toRadians(270)); // drive into second set of artifacts
//
//        TrajectoryActionBuilder goToShootSecondSet = driveIntoSecondSet.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(55,-15), Math.toRadians(270)); // go back after grabbing second set of artifacts to shoot

        TrajectoryActionBuilder goToWallSetAndDriveIn = goToShootFirstSet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(40,-65), Math.toRadians(-10)) // wall set
                .strafeToLinearHeading(new Vector2d(59,-65), Math.toRadians(-10)); // drive in

        TrajectoryActionBuilder goToShootWallSet = goToWallSetAndDriveIn.endTrajectory().endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(55,-15), Math.toRadians(270)); // go back after grabbing wall set

        //TODO: add trajectory to get off the luanch line
        TrajectoryActionBuilder goGetOffLaunchLine = goToShootWallSet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(52, -35), Math.toRadians(270));


        while (!opModeIsActive()){
            if (isStopRequested()){
                return;
            }

            telemetry.addData("Position during Init", initialPose);
            telemetry.update();
        }

//        SequentialAction pathingTest = new SequentialAction(
//                goToShootPreload.build(),
//                goToFirstSet.build(),
//                driveIntoFirstSet.build(),
//                goToShootFirstSet.build(),
//                goToSecondSet.build(),
//                driveIntoSecondSet.build(),
//                goToShootSecondSet.build(),
//                goToWallSetAndDriveIn.build(),
//                goToShootWallSet.build()
//        );

        ParallelAction shootPreload = new ParallelAction(
                // will keep flywheel always running for the action so parall with the sequential
                flywheel.runFlywheel(2060,3.3), //TODO: find working target velocity and finetune runnign time
                new SequentialAction(
                        new ParallelAction(
                                goToShootPreload.build(),
                                turret.aimTurret(-118,0.9) //TODO: find target position for turret, it is negative but find what value aims properly, can run the turret encoder test to find it
                        ),
                        stopper.disengageStopper(),
                        intake.holdIntakePower(-.8, 1.1)
                )
        );

        ParallelAction FirstBatch = new ParallelAction(
                flywheel.runFlywheel(2085,3), //TODO: find working target velocity and finetune runnign time
                new SequentialAction(
                        goToFirstSet.build(),
                        new ParallelAction(
                                intake.holdIntakePower(-0.8,2),
                                driveIntoFirstSet.build()
                        ),
                        goToShootFirstSet.build(),
                        stopper.disengageStopper(),
                        intake.holdIntakePower(-0.75,1.2)
                )

        );


//        ParallelAction SecondBatch = new ParallelAction(
//                flywheel.runFlywheel(2080,6), //TODO: find working target velocity and finetune runnign time
//                new SequentialAction(
//                        goToSecondSet.build(),
//                        new ParallelAction(
//                                intake.holdIntakePower(-0.8,2),
//                                driveIntoSecondSet.build()
//                        ),
//                        goToShootSecondSet.build(),
//                        stopper.disengageStopper(),
//                        intake.holdIntakePower(-0.75,2)
//                )
//
//        );

        ParallelAction WallBatch = new ParallelAction(
                flywheel.runFlywheel(2085,6), //TODO: find working target velocity and finetune runnign time
                new SequentialAction(
                        new ParallelAction(
                                intake.holdIntakePower(-0.8,3),
                                goToWallSetAndDriveIn.build()
                        ),
                        goToShootWallSet.build(),
                        stopper.disengageStopper(),
                        intake.holdIntakePower(-0.75,1.2),
                        stopper.engageStopper(),
                        new ParallelAction(
                                goGetOffLaunchLine.build(),
                                turret.aimTurret(0, .9)
                        )
                )

        );



        Actions.runBlocking(
                new SequentialAction(
                        shootPreload,
                        stopper.engageStopper(),
                        FirstBatch,
                        stopper.engageStopper(),
//                        SecondBatch,
//                        stopper.engageStopper(),
                        WallBatch,
                        stopper.engageStopper()
//                        goGetOffLaunchLine.build(),
//                        stopper.engageStopper()
                )


        );

    }
}
