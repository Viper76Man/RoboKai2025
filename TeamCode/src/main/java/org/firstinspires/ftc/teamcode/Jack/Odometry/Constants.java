package org.firstinspires.ftc.teamcode.Jack.Odometry;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants();
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(RobotConstantsV1.forwardPodY)
            .strafePodX(RobotConstantsV1.strafePodX)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(RobotConstantsV1.pinpointName)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                //.pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
