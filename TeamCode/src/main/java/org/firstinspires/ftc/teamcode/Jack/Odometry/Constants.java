package org.firstinspires.ftc.teamcode.Jack.Odometry;

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
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(RobotConstantsV1.robotMassKG)
            .forwardZeroPowerAcceleration(RobotConstantsV1.forwardZeroPowerAcceleration)    // was -18.4
            .lateralZeroPowerAcceleration(RobotConstantsV1.lateralZeroPowerAcceleration)    // was -18.4
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(RobotConstantsV1.centripetalScaling)
            .translationalPIDFCoefficients(RobotConstantsV1.translationalPIDCoefficients)  // was 0.3, 0, 0.01, 0
            .headingPIDFCoefficients(RobotConstantsV1.headingPIDFCoefficients)  // was 5, 0, 0, 0
            .drivePIDFCoefficients(RobotConstantsV1.drivePIDCoefficients);  // was 0.05, 0, 0, 0.6, 0


    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName(RobotConstantsV1.frontLeft)
            .leftRearMotorName(RobotConstantsV1.backLeft)
            .rightFrontMotorName(RobotConstantsV1.frontRight)
            .rightRearMotorName(RobotConstantsV1.backRight)
            .leftFrontMotorDirection(RobotConstantsV1.frontLeftDirection)
            .leftRearMotorDirection(RobotConstantsV1.backLeftDirection)
            .rightFrontMotorDirection(RobotConstantsV1.frontRightDirection)
            .rightRearMotorDirection(RobotConstantsV1.backRightDirection)
            .xVelocity(RobotConstantsV1.xVelocity)   // was 26.99
            .yVelocity(RobotConstantsV1.yVelocity)  // was 26.99
            .useBrakeModeInTeleOp(RobotConstantsV1.useBrakeInTeleOp);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(RobotConstantsV1.forwardPodY)
            .strafePodX(RobotConstantsV1.strafePodX)
            .distanceUnit(RobotConstantsV1.podsMeasurementUnit)
            .hardwareMapName(RobotConstantsV1.pinpointName)
            .yawScalar(1.0)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(RobotConstantsV1.forwardPodDirection)
            .strafeEncoderDirection(RobotConstantsV1.lateralPodDirection);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            500,
            0.25,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
