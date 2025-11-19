package org.firstinspires.ftc.teamcode.Auto;

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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Blue Far Side Auto..")
public class BlueFarSideAuto extends LinearOpMode {
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
                if (timer.seconds() > 0.4){
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
                if (timer.seconds() > 0.4){
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
            private final double power;

            public HoldIntakePower(double power) {
                this.power = power;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeMotor.setPower(power);
                packet.put("intakePower", power);
                // will return false so it keeps running
                return false;
            }
        }

        public Action holdIntakePower(double power) {
            return new HoldIntakePower(power);
        }

        // stop the intake
        public class StopIntake implements Action {
            private boolean done = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!done) {
                    intakeMotor.setPower(0);
                    done = true;
                }
                return true; // done after one call
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

            public HoldBeltPower(double power) {
                this.power = power;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                beltMotor.setPower(power);
                packet.put("belt power ", power);
                // will return false so it keeps running
                return false;
            }
        }

        public Action holdBeltPower(double power) {
            return new HoldBeltPower(power);
        }

        // stop the intake
        public class StopBelt implements Action {
            private boolean done = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!done) {
                    beltMotor.setPower(0);
                    done = true;
                }
                return true; // done after one call
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

            public HoldFlywheelVelocity(double velocity) {
                this.velocity = velocity;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                flywheel.setVelocity(velocity);
                packet.put("fly velocity ", velocity);
                // Driver Hub telemetry
                double actual = flywheel.getVelocity();
                telemetry.addData("Flywheel target", velocity);
                telemetry.addData("Flywheel actual", actual);
                telemetry.update();

                return false;
            }
        }

        public Action holdFlywheelVelocity(double vel) {
            return new HoldFlywheelVelocity(vel);
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



        Pose2d initialPose = new Pose2d(61, -10, Math.PI);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Flywheel flywheel = new Flywheel(hardwareMap, telemetry);
        Kicker kicker = new Kicker(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Belt belt = new Belt(hardwareMap);

        TrajectoryActionBuilder goToShootPreload = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(55,-15), Math.toRadians(205))
                .waitSeconds(4);  // position to shoot zero batch


        ParallelAction prepareToShootPreload = new ParallelAction(
                goToShootPreload.build(),
                flywheel.holdFlywheelVelocity(2750)
        );

        ParallelAction shootFirst = new ParallelAction(
                kicker.kickerUp()
        );

        ParallelAction GoAndShootPreLoad = new ParallelAction(
                new SequentialAction(
                        goToShootPreload.build(),
                        shootFirst,
                        kicker.kickerDown(),
                        kicker.kickerDown(),
                        kicker.kickerDown(),
                        kicker.kickerUp(), // shoot
                        kicker.kickerDown(),
                        kicker.kickerDown(),
                        kicker.kickerDown(),
                        kicker.kickerUp() // shoot
                ),
                belt.holdBeltPower(0.8),
                flywheel.holdFlywheelVelocity(2900)
        );




        while (!opModeIsActive()){
            telemetry.addData("Position during Init", initialPose);
            telemetry.update();
        }


        Actions.runBlocking(
                new SequentialAction(
                        prepareToShootPreload,
                        shootFirst,
                        new ParallelAction(
                                new SequentialAction(kicker.kickerDown(),
                                        kicker.kickerDown(),
                                        kicker.kickerDown(),
                                        kicker.kickerUp(), // shoot
                                        kicker.kickerDown(),
                                        kicker.kickerDown(),
                                        kicker.kickerDown(),
                                        kicker.kickerUp()
                                ),
                                belt.holdBeltPower(0.8)
                        )
                )
        );

    }
}
