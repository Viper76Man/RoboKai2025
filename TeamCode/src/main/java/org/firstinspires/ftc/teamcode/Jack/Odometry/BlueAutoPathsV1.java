package org.firstinspires.ftc.teamcode.Jack.Odometry;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

@Configurable
public class BlueAutoPathsV1 {

    public static Pose shootPose = new Pose(83.6, 17.4, Math.toRadians(70));
    public static Pose startPose = new Pose(87, 9.6, Math.toRadians(90));
    public static double toShootPoseTValue = 1;
    public Path outOfStart, toFirstArtifacts;

    //==============================================================================================
    public void buildPaths() {
        outOfStart = new Path(new BezierLine(startPose, shootPose));
        outOfStart.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());
    }
}
