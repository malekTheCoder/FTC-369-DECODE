package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Blue Far Side Auto")
public class BlueFarSideAuto extends LinearOpMode {
    public class Shooter{
        private DcMotorEx flywheel;
        private Servo servoLift;

        public Shooter(HardwareMap hardwareMap){

        } // constructor

    }

    public class Intake{
        private DcMotor intakeMotor;

        public Intake (HardwareMap hardwareMap){

        }// constructor


    }

    public class Belt{
        private DcMotor beltMotor;

        public Belt(HardwareMap hardwareMap){

        }// constructor

    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
