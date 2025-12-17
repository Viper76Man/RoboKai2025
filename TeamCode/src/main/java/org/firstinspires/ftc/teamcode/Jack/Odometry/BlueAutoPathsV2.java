package org.firstinspires.ftc.teamcode.Jack.Odometry;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Jack.Other.CustomPath;

@Configurable
public class BlueAutoPathsV2 {
    public static Pose startPoseFar = new Pose(87, 9.6, Math.toRadians(90));
    public static Pose shootPoseFar = new Pose(83.6, 17.4, Math.toRadians(70));



    public static double toShootPoseFarTValue = 1;
    public CustomPath outOfStartFar, toFirstArtifacts;

    //==============================================================================================
    public void buildPaths() {
        outOfStartFar = new CustomPath(startPoseFar, shootPoseFar, 1);
    }
}
