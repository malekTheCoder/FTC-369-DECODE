package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.703) // update with the mass of the robot in kilograms
            .forwardZeroPowerAcceleration(-44.152603477197)
            .lateralZeroPowerAcceleration(-67.0466955374)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02,0,0.00001,0.6,0.05))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.025,0,0.00005,0.6,0.01))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08,0,0.01,0.75))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.15,0,0.001,0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,0.01,0.06))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2.5,0,0.01,0.03))
            .centripetalScaling(0.0004);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.8, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)  // check on the motor directions
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(68.9566962775283)
            .yVelocity(53.37731453);


    // strafe x: 6.17598425 positive
    // forward y: 0.97322835 positive
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0.97322835) // set correct offsets
            .strafePodX(6.17598425)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD) // check directions for odo pods
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);



    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
