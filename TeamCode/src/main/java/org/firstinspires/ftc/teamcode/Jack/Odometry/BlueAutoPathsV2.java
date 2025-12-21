package org.firstinspires.ftc.teamcode.Jack.Odometry;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class BlueAutoPathsV2 {
    public static Pose startPoseFar = new Pose(62, 9.6, Math.toRadians(90));
    public static Pose shootPoseFar = new Pose(56, 15, Math.toRadians(110));

    public static Pose topArtifactsPickup = new Pose(47, 83.5, Math.toRadians(180));
    public static Pose middleArtifactsPickup = new Pose(47, 59.5, Math.toRadians(180));
    public static Pose bottomArtifactsPickup = new Pose(47, 35.5, Math.toRadians(180));


    public static Pose overdriveFirstBack = new Pose(56, 95.5, Math.toRadians(180));
    public static Pose ballsPickup1Back = new Pose(26, 35.5, Math.toRadians(180));
    public static Pose overdriveBackToShootBack = new Pose(84, -50, Math.toRadians(110));
    public static Pose backToShoot1StartPointBack = new Pose(33, 17, Math.toRadians(110));


    public static Pose overdriveSecondBack = new Pose(56, 119.5, Math.toRadians(180));
    public static Pose ballsPickup2Back = new Pose(26, 59.5, Math.toRadians(180));
    public static Pose overdriveBackToShoot2Back = new Pose(84, -50, Math.toRadians(110));
    public static Pose backToShoot2StartPointBack = new Pose(33, 41, Math.toRadians(110));


    public static double toShootPoseFarTValue = 0.97;
    public static double artifacts1TValue = 0.125;
    public static double pickup1TValue = 0.97;
    public static double backToShoot1OverdriveTValue = 0.2;


    public static CustomPath outOfStartFar, toFirstArtifacts, pickup1, overdriveBack1, backToShoot1;
    public static CustomPath toSecondArtifacts, pickup2, overdriveBack2;
    public void buildPaths() {
        outOfStartFar = new CustomPath(startPoseFar, shootPoseFar, toShootPoseFarTValue);
        outOfStartFar.setLinearHeadingInterpolationPath1(startPoseFar.getHeading(), shootPoseFar.getHeading());
        outOfStartFar.setName("outOfStartFar");
        toFirstArtifacts = new CustomPath(shootPoseFar, bottomArtifactsPickup, overdriveFirstBack, artifacts1TValue, 1);
        toFirstArtifacts.setLinearHeadingInterpolationPath1(shootPoseFar.getHeading(), overdriveFirstBack.getHeading());
        toFirstArtifacts.setLinearHeadingInterpolationPath2(overdriveFirstBack.getHeading(), bottomArtifactsPickup.getHeading());
        toFirstArtifacts.setName("toFirstArtifacts");
        pickup1 = new CustomPath(bottomArtifactsPickup, ballsPickup1Back, pickup1TValue);
        pickup1.setConstantHeadingInterpolationPath1(ballsPickup1Back.getHeading());
        pickup1.setName("pickup1");
        overdriveBack1 = new CustomPath(ballsPickup1Back, overdriveBackToShootBack, backToShoot1OverdriveTValue);
        overdriveBack1.setLinearHeadingInterpolationPath1(ballsPickup1Back.getHeading(), overdriveBackToShootBack.getHeading());
        overdriveBack1.setName("overdriveBack1");
        backToShoot1 = new CustomPath(backToShoot1StartPointBack, shootPoseFar, 1);
        backToShoot1.setLinearHeadingInterpolationPath1(backToShoot1StartPointBack.getHeading(), shootPoseFar.getHeading());
        backToShoot1.setName("backToShoot1");
        toSecondArtifacts = new CustomPath(shootPoseFar, middleArtifactsPickup, overdriveSecondBack, artifacts1TValue, 1);
        toSecondArtifacts.setLinearHeadingInterpolationPath1(shootPoseFar.getHeading(), overdriveSecondBack.getHeading());
        toSecondArtifacts.setName("toSecondArtifacts");
        pickup2 = new CustomPath(middleArtifactsPickup, ballsPickup2Back, pickup1TValue);
        pickup2.setConstantHeadingInterpolationPath1(ballsPickup2Back.getHeading());
    }
}
