package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseHoldController {

    private Pose2d targetPose = null;
    private boolean holding = false;

    // proportional values
    private PDController xPD = new PDController(0.3, 0.00);
    private PDController yPD = new PDController(0.3, 0.00);
    private PDController headingPD = new PDController(4,  0.00);


    // not using deadbands for now
    private double positionDeadband = 0;  // inches
    private double headingDeadbandRad = Math.toRadians(0);


    public void startHolding(Pose2d currentPose) {
        targetPose = currentPose;
        holding = true;
        xPD.reset();
        yPD.reset();
        headingPD.reset();
    }

    public void stopHolding() {
        holding = false;
        targetPose = null;
        xPD.reset();
        yPD.reset();
        headingPD.reset();
    }

    public boolean isHolding() {
        return holding && targetPose != null;
    }

    public Pose2d getTargetPose() {
        return targetPose;
    }

    public DriveCommand update(Pose2d currentPose, double dtSeconds) {
        if (!isHolding()) return new DriveCommand(0, 0, 0);

        // x and y eroor field realative
        double xError = targetPose.position.x - currentPose.position.x;
        double yError = targetPose.position.y - currentPose.position.y;

        //wrapped heading error
        double heading = currentPose.heading.toDouble();
        double headingError = angleWrap(targetPose.heading.toDouble() - heading);

        // deadbands
        if (Math.abs(xError) < positionDeadband) xError = 0;
        if (Math.abs(yError) < positionDeadband) yError = 0;
        if (Math.abs(headingError) < headingDeadbandRad) headingError = 0;

        // Convert field error -> robot error (robot-centric commands)
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double forwardError =  cos * xError + sin * yError;
        double strafeError = -sin * xError + cos * yError;

        double forwardCommand = xPD.update(forwardError, dtSeconds);
        double strafeCommand  = -yPD.update(strafeError, dtSeconds);
        double turnCommand = -headingPD.update(headingError, dtSeconds);


        return new DriveCommand(forwardCommand, strafeCommand, turnCommand);
    }

    private static double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }



    // class for robot centric drive commands to pass to drivetrain
    public static class DriveCommand {
        //robot centric
        public final double forward; //  +forward
        public final double strafe;  // +right
        public final double turn;    // +ccw

        public DriveCommand(double forward, double strafe, double turn) {
            this.forward = forward;
            this.strafe = strafe;
            this.turn = turn;
        }
    }


    // PD controller class
    private static class PDController {
        private double kP;
        private double kD;
        private double lastError = 0.0;
        private boolean notFirst = false;

        PDController(double kP, double kD) {
            this.kP = kP;
            this.kD = kD;
        }

        void reset() {
            notFirst = false;
            lastError = 0.0;
        }

        double update(double error, double dt) {
            double dTerm = 0.0;
            if (kD != 0.0 && notFirst && dt > 1e-4) {
                dTerm = (error - lastError) / dt;
            }
            lastError = error;
            notFirst = true;
            return (kP * error) + (kD * dTerm);
        }
    }
}