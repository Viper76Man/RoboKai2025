package org.firstinspires.ftc.teamcode.Jack.Odometry;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;


@Configurable
public class RedAutoPathsV1 {
    public static Pose PPGPose = new Pose(100, 83.5, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    public static Pose PGPPose = new Pose(100, 59.5, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    public static Pose GPPPose = new Pose(100, 35.5, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    public static Pose startPose = new Pose(57, 9.6, Math.toRadians(90));
    public static double toShootPoseTValue = 1;
    public static Pose shootPose = new Pose(60.4, 17.4, Math.toRadians(110));
    public Path outOfStart;

    public void build(){
        outOfStart = new Path(new BezierLine(startPose, shootPose));
        outOfStart.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());
    }
}
