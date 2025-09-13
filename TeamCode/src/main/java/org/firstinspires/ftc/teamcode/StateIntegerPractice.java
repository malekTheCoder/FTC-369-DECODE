package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous
public class StateIntegerPractice extends OpMode {

    enum State {
        WAIT_FOR_A,
        WAIT_FOR_B,
        WAIT_FOR_X,
        FINISHED
    }

    State state = State.WAIT_FOR_A;

    DcMotor motor;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        state = State.WAIT_FOR_A;


    }

    @Override
    public void loop() {
        telemetry.addData("current state", state);

        switch (state) {
            case WAIT_FOR_A:
                telemetry.addLine("to exit state press A");
                if (gamepad1.a) state = State.WAIT_FOR_B;
                break;

            case WAIT_FOR_B:
                telemetry.addLine("to exist state press B");
                if (gamepad1.b) state = State.WAIT_FOR_X;
                break;
            case WAIT_FOR_X:
                telemetry.addLine("to exist state press X");
                if (gamepad1.x){
                    state = State.FINISHED;
                }
                break;

            default:
                telemetry.addLine("auto state machine finished");
        }

    }
}
