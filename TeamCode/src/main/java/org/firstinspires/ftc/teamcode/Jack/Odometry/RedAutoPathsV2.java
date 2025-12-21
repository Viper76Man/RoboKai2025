package org.firstinspires.ftc.teamcode.Jack.Odometry;
import com.pedropathing.geometry.Pose;

public class RedAutoPathsV2 {
    public static Pose startPoseFar = AutoPathsV2.RedAutoPathsV2.startPoseFar;
    public static Pose shootPoseFar = AutoPathsV2.RedAutoPathsV2.shootPoseFar;

    public static Pose topArtifactsPickup = AutoPathsV2.RedAutoPathsV2.topArtifactsPickup;
    public static Pose middleArtifactsPickup = AutoPathsV2.RedAutoPathsV2.middleArtifactsPickup;
    public static Pose bottomArtifactsPickup = DecodeFieldLocalizer.mirrorPose(BlueAutoPathsV2.bottomArtifactsPickup);




    public static Pose overdriveFirstBack = DecodeFieldLocalizer.mirrorPose(BlueAutoPathsV2.overdriveFirstBack);
    public static Pose ballsPickup1Back = DecodeFieldLocalizer.mirrorPose(BlueAutoPathsV2.ballsPickup1Back);
    public static Pose overdriveBackToShootBack = DecodeFieldLocalizer.mirrorPose(BlueAutoPathsV2.overdriveBackToShoot2Back);
    public static Pose backToShoot1StartPointBack = DecodeFieldLocalizer.mirrorPose(BlueAutoPathsV2.backToShoot1StartPointBack);


    public static Pose overdriveSecondBack = DecodeFieldLocalizer.mirrorPose(BlueAutoPathsV2.overdriveSecondBack);
    public static Pose ballsPickup2Back = DecodeFieldLocalizer.mirrorPose(BlueAutoPathsV2.ballsPickup2Back);
    public static Pose overdriveBackToShoot2Back = DecodeFieldLocalizer.mirrorPose(BlueAutoPathsV2.overdriveBackToShoot2Back);
    public static Pose backToShoot2StartPointBack = DecodeFieldLocalizer.mirrorPose(BlueAutoPathsV2.backToShoot2StartPointBack);




    public static double toShootPoseFarTValue = BlueAutoPathsV2.toShootPoseFarTValue;
    public static double artifacts1TValue = BlueAutoPathsV2.artifacts1TValue;
    public static double pickup1TValue = BlueAutoPathsV2.pickup1TValue;
    public static double backToShoot1OverdriveTValue = BlueAutoPathsV2.backToShoot1OverdriveTValue;


    public static CustomPath outOfStartFar, toFirstArtifacts, pickup1, overdriveBack1, backToShoot1;
    public static CustomPath toSecondArtifacts;

    //==============================================================================================
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

    }
}
