package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FinalTurret {

    public enum Mode {
        AUTO,    // uses botErrorDeg -> UpdatedTurret.update() + aimPIDF()
        MANUAL   // uses stickX      -> UpdatedTurret.manual()
    }

    private final UpdatedTurret turret;
    private Mode mode = Mode.AUTO;

    // Inputs Robot provides
    private double botErrorDeg = 0.0;
    private double manualStickX = 0.0;

    public FinalTurret(HardwareMap hardwareMap) {
        turret = new UpdatedTurret(hardwareMap.get(DcMotorEx.class, "turret"));
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public Mode getMode() {
        return mode;
    }

// for auto mode
    public void setBotErrorDeg(double botErrorDeg) {
        this.botErrorDeg = botErrorDeg;
    }

// for manual mode
    public void setManualStickX(double stickX) {
        this.manualStickX = stickX;
    }

    public void resetPosition() {
        turret.resetPosition();
    }


    public void update(Telemetry telemetry) {
        if (mode == Mode.MANUAL) {
            turret.manual(manualStickX);
        } else {
            turret.update(botErrorDeg, telemetry);
            turret.aimPIDF();
        }
    }

    public boolean isOnTargetDeg(double degTolerance) {
        return turret.isOnTarget(degTolerance);
    }

    public double getCurrentTicks() {
        return turret.getCurrentTicks();
    }

    public double getTargetTicks() {
        return turret.getTargetTicks();
    }

    public double getTargetVelocityTicksPerSec() {
        return turret.getTargetVelocityTicksPerSec();
    }

    public TurretPID getPid() {
        return turret.getPid();
    }


    public void setPidCoefficients(double kP, double kI, double kD, double kS, double kV, double maxOutput) {
        turret.getPid().setCoefficients(kP, kI, kD, kS, kV, maxOutput);
    }


    public void setTargetFilterAlpha(double alpha) {
        // make low pass filter non final and implement later
    }
    public UpdatedTurret getTurretLogic() {
        return turret;
    }
}