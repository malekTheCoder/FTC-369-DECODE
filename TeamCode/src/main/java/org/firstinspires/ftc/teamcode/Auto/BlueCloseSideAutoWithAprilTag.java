package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "AprilTag Blue Close Side Auto")
public class BlueCloseSideAutoWithAprilTag extends LinearOpMode {

    public class LimelightClass{
        private Limelight3A limelight;
        private IMU imu;


        public LimelightClass (HardwareMap hardwareMap){
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.start();
            limelight.pipelineSwitch(0);
        }

        public LLResult getLatestResult(){
            return  limelight.getLatestResult();
        }

        public boolean hasValidTarget(){
            LLResult result = limelight.getLatestResult();
            return (result != null) && (result.isValid());
        }

        public double getTx(){
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()){
                return result.getTx();
            }

            return 0.0;
        }

        public double getTy(){
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()){
                return result.getTy();
            }

            return 0.0;
        }

        public Pose3D getBotPoseMT1(){
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()){
                return  result.getBotpose();
            }

            return null;
        }

        public Pose2d getFieldPose2dMegaTag1(){
            Pose3D pose = getBotPoseMT1();
            if (pose == null){
                return null;
            }

            double xMeters = pose.getPosition().x;
            double yMeters = pose.getPosition().y;

            double xInches = xMeters * 39.3701;
            double yInches = yMeters * 39.3701;

            double yawRadians = pose.getOrientation().getYaw(AngleUnit.RADIANS);

            Pose2d newPose = new Pose2d(xInches, yInches, yawRadians);

            return newPose;

        }
    }
    public class Kicker {
        double KICKER_DOWN_POSITION = 0;
        double KICKER_UP_POSITION = 1;
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
                if (timer.seconds() > 0.25){
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
                if (timer.seconds() > 0.25){
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

        //Pose2d initialPose = new Pose2d(-57, -44, Math.toRadians(233));
        Pose2d initialPose;

        LimelightClass limelight = new LimelightClass(hardwareMap);
        Pose2d llPoseStart = limelight.getFieldPose2dMegaTag1();

        initialPose = llPoseStart;




        while (!opModeIsActive()){
            initialPose = limelight.getFieldPose2dMegaTag1();

            if (initialPose != null){
                double xVal = initialPose.position.x;
                double yVal = initialPose.position.y;
                double headingRadians = initialPose.heading.toDouble();
                double headingDegrees = Math.toDegrees(headingRadians);

                telemetry.addData("X value", xVal);
                telemetry.addData("Y value", yVal);
                telemetry.addData("Heading (Radians)", headingRadians);
                telemetry.addData("Heading (Degrees)", headingDegrees);
            }


            telemetry.update();

            if (isStopRequested()) return;
        }



        initialPose = new Pose2d(-10, -14.5, Math.toRadians(180));


        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Flywheel flywheel = new Flywheel(hardwareMap, telemetry);
        Kicker kicker = new Kicker(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Belt belt = new Belt(hardwareMap);






        TrajectoryActionBuilder moveToShootPreload = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-25,-18),Math.toRadians(230)); // go to shoot preload

        TrajectoryActionBuilder goToFirstBatch = moveToShootPreload.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-1,-17), Math.toRadians(270)); // go to get first set of artifacts

        TrajectoryActionBuilder driveIntoFirstBatch = goToFirstBatch.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-1,-35), Math.toRadians(270)); // drive into first set of artifacts

        TrajectoryActionBuilder goToShootFirstBatch = driveIntoFirstBatch.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-25,-18),Math.toRadians(230)); // go to shoot first batch

        TrajectoryActionBuilder goToSecondBatch = goToShootFirstBatch.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(23,-17), Math.toRadians(270)); // go to second set of artifacts

        TrajectoryActionBuilder driveIntoSecondBatch = goToSecondBatch.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(23,-35), Math.toRadians(270)); // drive into second set of artifacts

        TrajectoryActionBuilder TurnAndPushClassifierGate = driveIntoSecondBatch.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(4,-38), Math.toRadians(180)) // push classifier gate
                .strafeToLinearHeading(new Vector2d(4,-58), Math.toRadians(180));

        TrajectoryActionBuilder goToShootSecondBatch = TurnAndPushClassifierGate.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-25,-18),Math.toRadians(240)); // go to shoot second batch

        TrajectoryActionBuilder goToThirdBatch = goToShootSecondBatch.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(35,-30), Math.toRadians(270)); // go to third set of artifacts

        TrajectoryActionBuilder driveIntoThirdBatch = goToThirdBatch.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(35,-50), Math.toRadians(270)); // drive into third set of artifacts

        TrajectoryActionBuilder goToShootThirdBatch = driveIntoThirdBatch.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-44,-28),Math.toRadians(240)); // go to shoot third batch


//        ParallelAction shootTripleClose = new ParallelAction(
//                new SequentialAction(
//                        new SleepAction(2.5),
//                        kicker.kickerUp(),
//                        kicker.kickerDown(),
//                        new SleepAction(0.5),
//                        kicker.kickerUp(),
//                        kicker.kickerDown(),
//                        new SleepAction(0.5),
//                        kicker.kickerUp(),
//                        kicker.kickerDown()
//                ),
//                flywheel.holdFlywheelVelocity(2000,5),
//                intake.holdIntakePower(0.4,5),
//                belt.holdBeltPower(-0.25,5)
//        );

        SequentialAction shootTripleClose = new SequentialAction(

                new ParallelAction(
                        new SequentialAction(
                                new SleepAction(0.2),
                                kicker.kickerUp(),
                                kicker.kickerDown(),
                                new SleepAction(0.35),
                                kicker.kickerUp(),
                                kicker.kickerDown()
                        ),
                        flywheel.holdFlywheelVelocity(2000,2),
                        intake.holdIntakePower(0.6,2),
                        belt.holdBeltPower(-0.35,2)
                ),

                new ParallelAction(
                        new SequentialAction(
                                new SleepAction(0.35),
                                kicker.kickerUp(),
                                kicker.kickerDown()
                        ),
                        flywheel.holdFlywheelVelocity(2000,1),
                        intake.holdIntakePower(0.9,1),
                        belt.holdBeltPower(-0.85,1)
                )
        );


        SequentialAction goAndShootPreload = new SequentialAction(
                new ParallelAction(
                        moveToShootPreload.build(),
                        flywheel.holdFlywheelVelocity(2000,1.5)
                ),

                shootTripleClose
        );

        SequentialAction goGrabFirstSetAndComeBackAndShoot = new SequentialAction(
                flywheel.stopFlywheel(0),
                intake.stopIntake(),
                belt.stopBelt(),
                goToFirstBatch.build(),
                new ParallelAction(
                        intake.holdIntakePower(0.6,1.75),
                        belt.holdBeltPower(-0.35,1.75),
                        driveIntoFirstBatch.build()
                ),

                new ParallelAction(
                        intake.holdIntakePower(0.1,1.5),
                        belt.holdBeltPower(0.2,1.5),
                        flywheel.holdFlywheelVelocity(2000,1.5),
                        goToShootFirstBatch.build()

                        ),


                new ParallelAction(
                        new SequentialAction(
                                new SleepAction(0.2),
                                kicker.kickerUp(),
                                kicker.kickerDown(),
                                new SleepAction(0.3),
                                kicker.kickerUp(),
                                kicker.kickerDown()
                        ),
                        flywheel.holdFlywheelVelocity(2000,2),
                        intake.holdIntakePower(0.6,2),
                        belt.holdBeltPower(-0.25,2)
                ),

                new ParallelAction(
                        new SequentialAction(
                                new SleepAction(0.5),
                                kicker.kickerUp(),
                                kicker.kickerDown()
                        ),
                        flywheel.holdFlywheelVelocity(2000,2),
                        intake.holdIntakePower(0.9,2),
                        belt.holdBeltPower(-0.75,2)
                )

        );

        SequentialAction goGrabSecondBatchAndPushGateAndComeBackAndShoot = new SequentialAction(
                flywheel.stopFlywheel(0),
                intake.stopIntake(),
                belt.stopBelt(),
                goToSecondBatch.build(),
                new ParallelAction(
                        intake.holdIntakePower(0.6,2),
                        belt.holdBeltPower(-0.35,2),
                        driveIntoSecondBatch.build()
                ),
                intake.stopIntake(),
                belt.stopBelt(),
                TurnAndPushClassifierGate.build(),
                new SleepAction(1),


                new ParallelAction(
                        goToShootSecondBatch.build(),
                        flywheel.holdFlywheelVelocity(2000,1.5)
                        ),


        new ParallelAction(
                new SequentialAction(
                        new SleepAction(0.2),
                        kicker.kickerUp(),
                        kicker.kickerDown(),
                        new SleepAction(0.35),
                        kicker.kickerUp(),
                        kicker.kickerDown()
                ),
                flywheel.holdFlywheelVelocity(2000,2),
                intake.holdIntakePower(0.6,2),
                belt.holdBeltPower(-0.35,2)
        ),

                new ParallelAction(
                        new SequentialAction(
                                new SleepAction(0.35),
                                kicker.kickerUp(),
                                kicker.kickerDown()
                        ),
                        flywheel.holdFlywheelVelocity(2000,1),
                        intake.holdIntakePower(0.9,1),
                        belt.holdBeltPower(-0.85,1)
                )


        );

        SequentialAction goGrabThirdBatchAndComeBackAndShoot = new SequentialAction(
                flywheel.stopFlywheel(0),
                intake.stopIntake(),
                belt.stopBelt(),
                goToThirdBatch.build(),
                new ParallelAction(
                        intake.holdIntakePower(0.6,3),
                        belt.holdBeltPower(-0.35,3),
                        driveIntoThirdBatch.build()
                ),
                intake.stopIntake(),
                belt.stopBelt(),
                goToShootThirdBatch.build(),
                shootTripleClose

        );













        Actions.runBlocking(
                new SequentialAction(
                        goAndShootPreload,
                        goGrabFirstSetAndComeBackAndShoot,
                        goGrabSecondBatchAndPushGateAndComeBackAndShoot
                        //goGrabThirdBatchAndComeBackAndShoot
                )


        );

    }
}
