package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Stopper {

    private Servo stopperServo;

    public static final double ENGAGED_POSITION = 0.6;
    public static final double DISENGAGED_POSITION = 0.5;


    public Stopper(HardwareMap hardwareMap) {
        stopperServo = hardwareMap.get(Servo.class, "stopper");
    }

    public void engageStopper() {
        stopperServo.setPosition(ENGAGED_POSITION);
    }

    public void disengageStopper() {
        stopperServo.setPosition(DISENGAGED_POSITION);
    }
}
