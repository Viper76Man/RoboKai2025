package org.firstinspires.ftc.teamcode.Jack.Odometry;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class BlueAutoPathsV2 {
    public static Pose startPoseFar = new Pose(62, 9.6, Math.toRadians(90));
    public static Pose shootPoseFar = new Pose(56, 15, Math.toRadians(110));
    //100,35.5
    public static Pose closestArtifactsPickup = new Pose(44, 35.5, Math.toRadians(180));
    public static Pose overdriveFirst = new Pose(54, 55.5, Math.toRadians(180));



    public static double toShootPoseFarTValue = 1;
    public static double artifacts1TValue = 0.4;


    public static CustomPath outOfStartFar, toFirstArtifacts;

    //==============================================================================================
    public void buildPaths() {
        outOfStartFar = new CustomPath(startPoseFar, shootPoseFar, toShootPoseFarTValue);
        toFirstArtifacts = new CustomPath(shootPoseFar, closestArtifactsPickup, overdriveFirst, artifacts1TValue, 1);
    }
}
