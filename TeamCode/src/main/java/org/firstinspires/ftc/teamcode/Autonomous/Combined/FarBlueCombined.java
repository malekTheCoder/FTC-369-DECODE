package org.firstinspires.ftc.teamcode.Autonomous.Combined;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Far Blue Combined")
public class FarBlueCombined extends LinearOpMode {

    public class Hood{
        private double HOOD_ENGAGED_POSITION = 0.61;
        private double HOOD_DISENGAGED_POSITION = 0.43;
        private Servo hood;

        private ElapsedTime hoodTimer;

        public Hood(HardwareMap hardwareMap) {
            hood = hardwareMap.get(Servo.class, "hood");
            hood.setPosition(HOOD_ENGAGED_POSITION);
        }


    }
    public class Kicker {
        double KICKER_DOWN_POSITION = 0.2 ;
        double KICKER_UP_POSITION = 0.61;
        double INTAKE_POWER = 0.6;
        private DcMotorEx flywheel;
        private Servo kicker;
        private ElapsedTime timer = new ElapsedTime();


        public Kicker(HardwareMap hardwareMap){
            flywheel = hardwareMap.get(DcMotorEx.class, "fly");
            kicker = hardwareMap.get(Servo.class, "kicker");
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            kicker.setPosition(KICKER_DOWN_POSITION);


        } // constructor

        public class KickerUp implements Action{
            public boolean isReset = false;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!isReset){
                    isReset = true;
                    timer.reset();
                }

                kicker.setPosition(KICKER_UP_POSITION);
                if (timer.seconds() > 0.2){
                    return false;
                }

                return true;
            }
        }
        public Action kickerUp(){
            return new KickerUp();
        }


        public class KickerDown implements Action{
            public boolean isReset = false;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!isReset){
                    isReset = true;
                    timer.reset();
                }

                kicker.setPosition(KICKER_DOWN_POSITION);
                if (timer.seconds() > 0.2){
                    return false;
                }

                return true;
            }
        }
        public Action kickerDown(){
            return new KickerDown();
        }


    }

    public class Intake{
        private DcMotor intakeMotor;

        public Intake (HardwareMap hardwareMap){
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

    public class Belt{
        private DcMotor beltMotor;

        public Belt(HardwareMap hardwareMap){
            beltMotor = hardwareMap.get(DcMotor.class, "belt");
            beltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            beltMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }// constructor

        public class HoldBeltPower implements Action {
            private final double power;
            private double duration;
            private ElapsedTime beltTimer = new ElapsedTime();

            private boolean beltTimerStarted = false;

            public HoldBeltPower(double power, double duration) {
                this.power = power;
                this.duration = duration;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!beltTimerStarted){
                    beltTimer.reset();
                    beltTimerStarted = true;
                }

                beltMotor.setPower(power);
                packet.put("belt power ", power);


                if (beltTimer.seconds() > duration){
                    return false;
                } else {
                    return true;
                }

            }
        }

        public Action holdBeltPower(double power, double duration) {
            return new HoldBeltPower(power, duration);
        }

        // stop the intake
        public class StopBelt implements Action {


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                beltMotor.setPower(0);
                return false; // done after one call
            }
        }

        public Action stopBelt() {
            return new StopBelt();
        }

    }

    public class Flywheel{
        private DcMotorEx flywheel;
        private Telemetry telemetry;


        public Flywheel(HardwareMap hardwareMap, Telemetry telemetry){
            flywheel = hardwareMap.get(DcMotorEx.class, "fly");
            flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            this.telemetry = telemetry;
        }

        public double getVelocity(){
            return flywheel.getVelocity();
        }

        public class HoldFlywheelVelocity implements Action {
            private double velocity;

            private double duration;
            private ElapsedTime flywheelTimer = new ElapsedTime();

            private boolean flywheelTimerStarted = false;

            public HoldFlywheelVelocity(double velocity, double duration) {
                this.velocity = velocity;
                this.duration = duration;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!flywheelTimerStarted){
                    flywheelTimer.reset();
                    flywheelTimerStarted = true;
                }

                flywheel.setVelocity(velocity);

                packet.put("fly velocity ", velocity);
                // Driver Hub telemetry
                double actual = flywheel.getVelocity();
                telemetry.addData("Flywheel target", velocity);
                telemetry.addData("Flywheel actual", actual);
                telemetry.update();

                if (flywheelTimer.seconds() > duration){
                    return false;
                } else {
                    return true;
                }
            }
        }

        public Action holdFlywheelVelocity(double vel, double duration) {
            return new HoldFlywheelVelocity(vel, duration);
        }


        public class StopFlywheel implements Action {
            private final double velocity;

            public StopFlywheel(double velocity) {
                this.velocity = velocity;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                flywheel.setVelocity(velocity);
                packet.put("fly velocity ", velocity);

                return false;
            }
        }

        public Action stopFlywheel(double vel) {
            return new StopFlywheel(vel);
        }



    }

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(61, -15, Math.PI);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Flywheel flywheel = new Flywheel(hardwareMap, telemetry);
        Kicker kicker = new Kicker(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Belt belt = new Belt(hardwareMap);
        Hood hood = new Hood(hardwareMap);

        TrajectoryActionBuilder goToShootPreload = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(55,-19), Math.toRadians(201.5));


        TrajectoryActionBuilder goLoopForFirstSet = goToShootPreload.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(46,-23), Math.toRadians(270)) // go to first set of artifacts
                .strafeToLinearHeading(new Vector2d(46,-40), Math.toRadians(270)) // drive into first set of artifacts
                .strafeToLinearHeading(new Vector2d(54,-19), Math.toRadians(204)); // go back after grabbing first set of artifacts to shoot

        TrajectoryActionBuilder goLoopForGateBatch = goLoopForFirstSet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(34,-73), Math.toRadians(180)) // go to intake from gate
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(54,-19), Math.toRadians(202.5)); // go back to shoot

        TrajectoryActionBuilder goLoopForExtraRandom = goLoopForGateBatch.endTrajectory().endTrajectory()
                .strafeToLinearHeading(new Vector2d(40,-70), Math.toRadians(180))// go grab anywehre
                .strafeToLinearHeading(new Vector2d(20,-70), Math.toRadians(180))// go grab anywehre
                .strafeToLinearHeading(new Vector2d(55,-15), Math.toRadians(195)); // go to shoot


        SequentialAction shootPreload = new SequentialAction(
                new ParallelAction(
                        goToShootPreload.build(),
                        flywheel.holdFlywheelVelocity(2060, 0.75)
                ),
                new ParallelAction(
                        flywheel.holdFlywheelVelocity(2060,1.2),
                        intake.holdIntakePower(0.5,1.2),
                        belt.holdBeltPower(-0.5,1.2),
                        new SequentialAction(
                                new SleepAction(0.4),
                                kicker.kickerUp(),
                                kicker.kickerDown(),
                                new SleepAction(0.4),
                                kicker.kickerUp(),
                                kicker.kickerDown(),
                                new SleepAction(0.4),
                                kicker.kickerUp(),
                                kicker.kickerDown()
                        )
                )
        );

        SequentialAction firstBatchLoop = new SequentialAction(
                flywheel.stopFlywheel(0),
                new ParallelAction(
                        goLoopForFirstSet.build(),
                        intake.holdIntakePower(0.6,2),
                        belt.holdBeltPower(-0.8,2),
                        new SequentialAction(
                                new SleepAction(2.2),
                                flywheel.holdFlywheelVelocity(2060,0.3)
                        )
                ),
                new ParallelAction(
                        flywheel.holdFlywheelVelocity(2060,1.2),
                        intake.holdIntakePower(0.5,1.2),
                        belt.holdBeltPower(-0.5,1.2),
                        new SequentialAction(
                                new SleepAction(0.4),
                                kicker.kickerUp(),
                                kicker.kickerDown(),
                                new SleepAction(0.4),
                                kicker.kickerUp(),
                                kicker.kickerDown(),
                                new SleepAction(0.4),
                                kicker.kickerUp(),
                                kicker.kickerDown()
                        )
                )




        );

        SequentialAction gateBatchLoop = new SequentialAction(
                flywheel.stopFlywheel(0),
                new ParallelAction(
                        goLoopForGateBatch.build(),
                        flywheel.stopFlywheel(0),
                        intake.holdIntakePower(0.6,3.5),
                        belt.holdBeltPower(-0.8,3.5)
//                        new SequentialAction(
//                                new SleepAction(4.2),
//                                flywheel.holdFlywheelVelocity(2060,0.1)
//                        )
                ),
                new ParallelAction(
                        flywheel.holdFlywheelVelocity(2060,1.2),
                        intake.holdIntakePower(0.5,1.2),
                        belt.holdBeltPower(-0.4,1.2),
                        new SequentialAction(
                                new SleepAction(0.7),
                                kicker.kickerUp(),
                                kicker.kickerDown(),
                                new SleepAction(0.35),
                                kicker.kickerUp(),
                                kicker.kickerDown(),
                                new SleepAction(0.4),
                                kicker.kickerUp(),
                                kicker.kickerDown()
                        )
                )
        );












        while (!opModeIsActive()){
            telemetry.addData("Position during Init", initialPose);
            telemetry.update();
        }


        Actions.runBlocking(
                new SequentialAction(
//                        goToShootPreload.build(),
//                        goLoopForFirstSet.build(),
//                        goLoopForGateBatch.build()
                        shootPreload,
                        firstBatchLoop,
                        gateBatchLoop
                )


        );

    }
}
