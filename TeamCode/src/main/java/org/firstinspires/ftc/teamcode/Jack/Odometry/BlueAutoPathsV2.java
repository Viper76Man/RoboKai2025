package org.firstinspires.ftc.teamcode.Jack.Odometry;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class BlueAutoPathsV2 {
    public static Pose startPoseClose = new Pose(24, 125, Math.toRadians(140));
    public static Pose shootPoseClose = new Pose(55, 100, Math.toRadians(140));
    //BACK=-----------------------------------------------------------------------------------------
    public static Pose startPoseFar = new Pose(62, 9.6, Math.toRadians(90));
    public static Pose shootPoseFar = new Pose(56, 15, Math.toRadians(110));

    public static Pose topArtifactsPickup = new Pose(47, 83.5, Math.toRadians(180));
    public static Pose middleArtifactsPickup = new Pose(47, 56.5, Math.toRadians(180));
    public static Pose bottomArtifactsPickup = new Pose(47, 35.5, Math.toRadians(180));


    public static Pose overdriveFirstBack = new Pose(56, 95.5, Math.toRadians(180));
    public static Pose ballsPickup1Back = new Pose(20, 35.5, Math.toRadians(180));
    public static Pose overdriveBackToShootBack = new Pose(84, -50, Math.toRadians(110));
    public static Pose backToShoot1StartPointBack = new Pose(35, 17, Math.toRadians(110));


    public static Pose overdriveSecondBack = new Pose(56, 119.5, Math.toRadians(180));
    public static Pose ballsPickup2Back = new Pose(20, 56.5, Math.toRadians(180));
    public static Pose overdriveBackToShoot2Back = new Pose(84, -50, Math.toRadians(110));
    public static Pose backToShoot2StartPointBack = new Pose(35, 34, Math.toRadians(110));

    public static Pose overdriveThirdBack = new Pose(56, 143.5, Math.toRadians(180));
    public static Pose ballsPickup3Back = new Pose(20, 80.5, Math.toRadians(180));
    public static Pose overdriveBackToShoot3Back = new Pose(84, -60, Math.toRadians(110));
    public static Pose backToShoot3StartPointBack = new Pose(35, 34, Math.toRadians(110));


    public static double toShootPoseFarTValue = 0.95;
    public static double artifacts1TValue = 0.125;
    public static double pickup1TValue = 0.78;
    public static double backToShoot1OverdriveTValue = 0.17;

    public static double artifacts2TValue = 0.105;
    public static double backToShoot2OverdriveTValue = 0.22;

    public static double artifacts3TValue = 0.125;
    public static double backToShoot3OverdriveTValue = 0.277;



    //CLOSE-----------------------------------------------------------------------------------------
    public static double toShootPoseCloseTValue = 0.95;
    public static double artifacts1TValueClose = 0.125;
    public static double pickup1TValueClose = 0.78;
    public static double forwardToShoot1OverdriveTValue = 0.15;

    public static double artifacts2TValueClose = 0.105;
    public static double forwardToShoot2OverdriveTValue = 0.2;



    public static CustomPath outOfStartClose, toFirstArtifactsClose, pickup1Close, overdriveForward1Close, backToShoot1Close;
    public static CustomPath toSecondArtifactsClose, pickup2Close, overdriveForward2Close, backToShoot2Close;


    public static CustomPath outOfStartFar, toFirstArtifacts, pickup1, overdriveBack1, backToShoot1;
    public static CustomPath toSecondArtifacts, pickup2, overdriveBack2, backToShoot2;
    public static CustomPath toThirdArtifacts, pickup3, overdriveBack3, backToShoot3;
    public void buildPaths() {

        outOfStartClose = new CustomPath(startPoseClose, shootPoseClose, toShootPoseCloseTValue);
        outOfStartClose.setConstantHeadingInterpolationPath1(shootPoseClose.getHeading());
        outOfStartClose.setName("outOfStartClose");
        //BACK--------------------------------------------------------------------------------------
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
        backToShoot1 = new CustomPath(backToShoot1StartPointBack, shootPoseFar, 0.95);
        backToShoot1.setLinearHeadingInterpolationPath1(backToShoot1StartPointBack.getHeading(), shootPoseFar.getHeading());
        backToShoot1.setName("backToShoot1");

        toSecondArtifacts = new CustomPath(shootPoseFar, middleArtifactsPickup, overdriveSecondBack, artifacts2TValue, 1);
        toSecondArtifacts.setLinearHeadingInterpolationPath1(shootPoseFar.getHeading(), overdriveSecondBack.getHeading());
        toSecondArtifacts.setName("toSecondArtifacts");
        pickup2 = new CustomPath(middleArtifactsPickup, ballsPickup2Back, pickup1TValue);
        pickup2.setConstantHeadingInterpolationPath1(ballsPickup2Back.getHeading());
        pickup2.setName("pickup2");
        overdriveBack2 = new CustomPath(ballsPickup2Back, overdriveBackToShoot2Back, backToShoot2OverdriveTValue);
        overdriveBack2.setLinearHeadingInterpolationPath1(ballsPickup1Back.getHeading(), overdriveBackToShootBack.getHeading());
        overdriveBack2.setName("overdriveBack2");
        backToShoot2 = new CustomPath(backToShoot2StartPointBack, shootPoseFar, 0.95);
        backToShoot2.setLinearHeadingInterpolationPath1(backToShoot2StartPointBack.getHeading(), shootPoseFar.getHeading());
        backToShoot2.setName("backToShoot2");

        toThirdArtifacts = new CustomPath(shootPoseFar, topArtifactsPickup, overdriveThirdBack, artifacts3TValue, 1);
        toThirdArtifacts.setLinearHeadingInterpolationPath1(shootPoseFar.getHeading(), overdriveThirdBack.getHeading());
        toThirdArtifacts.setName("toThirdArtifacts");
        pickup3 = new CustomPath(topArtifactsPickup, ballsPickup3Back, pickup1TValue);
        pickup3.setConstantHeadingInterpolationPath1(ballsPickup3Back.getHeading());
        pickup3.setName("pickup3");
        overdriveBack3 = new CustomPath(ballsPickup3Back, overdriveBackToShoot3Back, backToShoot3OverdriveTValue);
        overdriveBack3.setLinearHeadingInterpolationPath1(ballsPickup3Back.getHeading(), overdriveBackToShoot3Back.getHeading());
        overdriveBack3.setName("overdriveBack3");
        backToShoot3 = new CustomPath(backToShoot3StartPointBack, shootPoseFar, 0.95);
        backToShoot3.setLinearHeadingInterpolationPath1(backToShoot3StartPointBack.getHeading(), shootPoseFar.getHeading());
        backToShoot3.setName("backToShoot3");
    }
}
