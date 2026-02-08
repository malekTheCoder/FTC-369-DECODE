package org.firstinspires.ftc.teamcode.Final.Autonomous.Solo;

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
import org.firstinspires.ftc.teamcode.Subsystems.PoseStorage;

@Autonomous(name = "RedClose 2 row")
public class RedClose2Row extends LinearOpMode {

    MecanumDrive drive;

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

                if (Math.abs(turret.getCurrentPosition() - targetPosition) < 3){
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
        private double engagedPosition = 0.7; // fine tune this value
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

    public class Update implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            Pose2d pose = drive.localizer.getPose();
            PoseStorage.savedPose = pose;

            // Keep running until RaceAction ends because the main sequence finished.
            return true;
        }
    }
    public Action updatePose(){
        return new Update();
    }


    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(-61,41, Math.toRadians(90)); // initial pose from meep meep

        PoseStorage.pinpointHeadingOffsetDriverRelative = -90;

        drive = new MecanumDrive(hardwareMap, initialPose);
        Turret turret = new Turret(hardwareMap);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Stopper stopper = new Stopper(hardwareMap);




        TrajectoryActionBuilder goToShootPreload = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-10,19), Math.toRadians(90)); // position to shoot zero batch

//        TrajectoryActionBuilder goToFirstSet = goToShootPreload.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(-15,32), Math.toRadians(90)); // go to first set of artifacts

        TrajectoryActionBuilder driveIntoFirstSet = goToShootPreload.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-10,53), Math.toRadians(90)); // drive into first set of artifacts

        TrajectoryActionBuilder goEmptyGate = driveIntoFirstSet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(0, 58), Math.toRadians(195));

        TrajectoryActionBuilder goToShootFirstSet = goEmptyGate.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-10,18), Math.toRadians(90)); // go back after grabbing first set of artifacts to shoot

        TrajectoryActionBuilder goToSecondSet = goToShootFirstSet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(14,32), Math.toRadians(90)); // go to second set of artifacts

        TrajectoryActionBuilder driveIntoSecondSet = goToSecondSet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(14,53), Math.toRadians(90)); // drive into second set of artifacts

        TrajectoryActionBuilder goToShootSecondSet = driveIntoSecondSet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-8,20), Math.toRadians(90)); // go back after grabbing second set of artifacts to shoot

        TrajectoryActionBuilder goToThirdSet = goToShootSecondSet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(40,32), Math.toRadians(90)); // go to third set of artifacts

        TrajectoryActionBuilder driveIntoThirdSet = goToThirdSet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(40,53), Math.toRadians(90)); // drive into third set of artifacts

        TrajectoryActionBuilder goToShootThirdSet = driveIntoThirdSet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-8,20), Math.toRadians(90)); // go back after grabbing third set of artifacts to shoot

        TrajectoryActionBuilder goGetOffLaunchLine = goToShootSecondSet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(0,38),Math.toRadians(0)); // go shoot second batch



//        SequentialAction pathingTest = new SequentialAction(
//                goToShootPreload.build(),
//                goToFirstSet.build(),
//                driveIntoFirstSet.build(),
//                goToShootFirstSet.build(),
//                goToSecondSet.build(),
//                driveIntoSecondSet.build(),
//                goToShootSecondSet.build(),
//                goToThirdSet.build(),
//                driveIntoThirdSet.build(),
//                goToShootThirdSet.build(),
//                goGetOffLaunchLine.build()
//        );

        ParallelAction shootPreload = new ParallelAction(
                // will keep flywheel always running for the action so parall with the sequential
                flywheel.runFlywheel(1730,4), //TODO: find working target velocity and finetune runnign time, this running time should basically be the whole action so make sure its long enough, sytart with a long time and reduce from there
                new SequentialAction(
                        new ParallelAction(
                                turret.aimTurret(-783,0.9), //TODO: find target position for turret, it is negative but find what value aims properly, can run the turret encoder test to find it
                                goToShootPreload.build()

                        ),

                        stopper.disengageStopper(),
                        intake.holdIntakePower(-0.75,2) //TODO fine tune
                )
        );

        SequentialAction FirstBatch = new SequentialAction(
                new ParallelAction(
                        intake.holdIntakePower(-0.85, 1.7), //TODO fine tune,
                        driveIntoFirstSet.build()
                ),
                intake.stopIntake(),
                goEmptyGate.build(),
                new ParallelAction(
                        flywheel.runFlywheel(1740,4),
                        new SequentialAction(
                                goToShootFirstSet.build(),
                                stopper.disengageStopper(),
                                intake.holdIntakePower(-0.75, 2)
                        )

                )

        );

        SequentialAction SecondBatch = new SequentialAction(
                goToSecondSet.build(),
                new ParallelAction(
                        intake.holdIntakePower(-0.85, 1.5), //TODO fine tune,
                        driveIntoSecondSet.build()
                ),
                new ParallelAction(
                        flywheel.runFlywheel(1760,3.7),
                        new SequentialAction(
                                goToShootSecondSet.build(),
                                stopper.disengageStopper(),
                                intake.holdIntakePower(-0.75, 2)
                        )

                )

        );

        SequentialAction ThirdBatch = new SequentialAction(
                goToThirdSet.build(),
                new ParallelAction(
                        intake.holdIntakePower(-0.8, 1.5), //TODO fine tune,
                        driveIntoThirdSet.build(),
                        turret.aimTurret(-789, 0.9)
                ),
                new ParallelAction(
                        flywheel.runFlywheel(1790,4.7), //TODO this flywheel timer wont be the same for all batvhes it will have to get longer since the path to get to the tshooting spot gets longer
                        new SequentialAction(
                                goToShootThirdSet.build(),
                                stopper.disengageStopper(),
                                intake.holdIntakePower(-0.75, 2),
                                turret.aimTurret(0, .9)
                        )

                )

        );



        while (!opModeIsActive()){
            if (isStopRequested()){
                return;
            }

            telemetry.addData("Position during Init", initialPose);
            telemetry.update();
        }



        // TODO when testing go step by step, comment out all but the ffirst and then incremmentallg uncomment the next line
        // TODO will make testing and troubleshooting easier
        Actions.runBlocking(
                new ParallelAction(

                        updatePose(),

                new SequentialAction(
                        shootPreload,
                        stopper.engageStopper(),
                        FirstBatch,
                        stopper.engageStopper(),
                        SecondBatch,
                        stopper.engageStopper(),
//                        ThirdBatch,
//                        stopper.engageStopper(),
                        new ParallelAction(
                                goGetOffLaunchLine.build(),
                                stopper.engageStopper(),
                                turret.aimTurret(0, .9),
                                intake.stopIntake()
                        )

//                        goToShootPreload.build(),
//                        goToFirstSet.build(),
//                        driveIntoFirstSet.build(),
//                        goToShootFirstSet.build(),
//                        goToSecondSet.build(),
//                        driveIntoSecondSet.build(),
//                        goToShootSecondSet.build(),
//                        goToThirdSet.build(),
//                        driveIntoThirdSet.build(),
//                        goToShootThirdSet.build(),
//                        goGetOffLaunchLine.build()
                )
                )

        );

    }
}