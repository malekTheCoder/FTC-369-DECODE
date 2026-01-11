package org.firstinspires.ftc.teamcode.Auton.Solo;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.Combined.FarBlueCombined;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "BlueCloseSolo")
public class BlueCloseSolo extends LinearOpMode {

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
        // to be implemented using outtake subsystem
    }





    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(-50, -49, Math.toRadians(234)); // initial pose from meep meep
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Turret turret = new Turret(hardwareMap);




        TrajectoryActionBuilder goToShootPreload = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-9,-10), Math.toRadians(270)); // position to shoot zero batch

        TrajectoryActionBuilder goToFirstBatchAndDriveInAndGoBackToShoot = goToShootPreload.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-9,-20), Math.toRadians(270)) // go to first set of artifacts
                .strafeToLinearHeading(new Vector2d(-9,-45), Math.toRadians(270)) // drive into first set of artifacts
                .strafeToLinearHeading(new Vector2d(-9,-10), Math.toRadians(270)); // go back after grabbing first set of artifacts to shoot

        TrajectoryActionBuilder goLoopPathForSecondBatch = goToFirstBatchAndDriveInAndGoBackToShoot.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(15,-20), Math.toRadians(270)) // go to second set of artifacts
                .strafeToLinearHeading(new Vector2d(15,-45), Math.toRadians(270)) // drive into second set of artifacts
                .strafeToLinearHeading(new Vector2d(-9,-10), Math.toRadians(270)); // go back after grabbing second set of artifacts to shoot


        TrajectoryActionBuilder goLoopPathForThirdBatch = goLoopPathForSecondBatch.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(40,-20), Math.toRadians(270)) // go to third set of artifacts
                .strafeToLinearHeading(new Vector2d(40,-45), Math.toRadians(270)) // drive into third set of artifacts
                .strafeToLinearHeading(new Vector2d(-9,-10), Math.toRadians(270)); // go back after grabbing third set of artifacts to shoot


        TrajectoryActionBuilder goGetOffLaunchLine = goLoopPathForThirdBatch.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-9,-20),Math.toRadians(224)); // go shoot second batch
        while (!opModeIsActive()){
            if (isStopRequested()){
                return;
            }

            telemetry.addData("Position during Init", initialPose);
            telemetry.update();
        }


        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                goToShootPreload.build(),
                                turret.aimTurret(-150,0.9)
                        )

//                        goToFirstBatchAndDriveInAndGoBackToShoot.build(),
//                        goLoopPathForSecondBatch.build(),
//                        goLoopPathForThirdBatch.build(),
//                        goGetOffLaunchLine.build()
                        //kicker.kickerDown()

                )

        );

    }
}
