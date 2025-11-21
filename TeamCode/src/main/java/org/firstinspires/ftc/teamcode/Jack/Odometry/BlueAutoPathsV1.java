package org.firstinspires.ftc.teamcode.Jack.Odometry;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

public class BlueAutoPathsV1 {
    public Pose startPose = new Pose(57, 9.6, Math.toRadians(90));
    public double toShootPoseTValue = 1;
    public Pose shootPose = new Pose(60.4, 17.4, Math.toRadians(110));

    public Path outOfStart, toFirstArtifacts;

    //==============================================================================================
    public void buildPaths() {
        outOfStart = new Path(new BezierLine(startPose, shootPose));
        outOfStart.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());
    }
}
