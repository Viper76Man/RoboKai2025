package org.firstinspires.ftc.teamcode.Jack.Odometry;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Drive.Robot;

import java.io.ObjectStreamClass;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class AutoPathsV2 {
    public List<Points> pointsList = new ArrayList<>();
    private List<Pose> posesList = new ArrayList<>();
    private List<Double> tValuesList = new ArrayList<>();
    public enum Points {
        START_BACK,
        START_FRONT,
        SHOOT_BACK,
        SHOOT_FRONT,
        ARTIFACTS_FRONT_BLUE,
        ARTIFACTS_MIDDLE_BLUE,
        ARTIFACTS_BACK_BLUE,
        ARTIFACTS_FRONT_RED,
        ARTIFACTS_MIDDLE_RED,
        ARTIFACTS_BACK_RED,
        HUMAN_PLAYER_RED,
        HUMAN_PLAYER_BLUE
    }

    //BLUE------------------------------------------------------------------------------------------
    @Configurable
    static class BlueAutoPathsV2 {
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
    }
    //RED-------------------------------------------------------------------------------------------
    static class RedAutoPathsV2 {
        public static Pose startPoseFar = DecodeFieldLocalizer.mirrorPose(BlueAutoPathsV2.startPoseFar);
        public static Pose shootPoseFar = DecodeFieldLocalizer.mirrorPose(BlueAutoPathsV2.shootPoseFar);

        public static Pose topArtifactsPickup = DecodeFieldLocalizer.mirrorPose(BlueAutoPathsV2.topArtifactsPickup);
        public static Pose middleArtifactsPickup = DecodeFieldLocalizer.mirrorPose(BlueAutoPathsV2.middleArtifactsPickup);
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
    }
    //----------------------------------------------------------------------------------------------
    public void setNextPoint(Points point){
        pointsList.add(point);
    }

    public void setNextPoints(Points... points){
        pointsList.addAll(Arrays.asList(points));
    }

    public void addFunction(Runnable action){
       action.run();
    }

    public void addPose(Pose point, double... maxTValues){
        posesList.add(point);
        for (double t : maxTValues) {
            tValuesList.add(t);
        }
    }

    public void setNextPoints(List<Points> pointsList_){
        pointsList.addAll(pointsList_);
    }


    public void build(Robot.Alliance alliance){
        for(Points p : pointsList){
            switch (p){
                case START_BACK:
                    switch (alliance){
                        case RED:
                            addPose(RedAutoPathsV2.startPoseFar, 1);
                            break;
                        case BLUE:
                            addPose(BlueAutoPathsV2.startPoseFar,1);
                            break;
                    }
                    break;
                case SHOOT_BACK:
                    switch (alliance){
                        case RED:
                            addPose(RedAutoPathsV2.shootPoseFar, RedAutoPathsV2.toShootPoseFarTValue);
                            break;
                        case BLUE:
                            addPose(BlueAutoPathsV2.shootPoseFar, BlueAutoPathsV2.toShootPoseFarTValue);
                            break;
                    }
                    break;
                case ARTIFACTS_BACK_BLUE:
                    addPose(BlueAutoPathsV2.bottomArtifactsPickup);
                    break;
                case ARTIFACTS_MIDDLE_BLUE:
                    addPose(BlueAutoPathsV2.middleArtifactsPickup);
                    break;
                case ARTIFACTS_FRONT_BLUE:
                    addPose(BlueAutoPathsV2.topArtifactsPickup);
                    break;
                case ARTIFACTS_BACK_RED:
                    addPose(RedAutoPathsV2.bottomArtifactsPickup);
                    break;
                case ARTIFACTS_MIDDLE_RED:
                    addPose(RedAutoPathsV2.middleArtifactsPickup);
                    break;
                case ARTIFACTS_FRONT_RED:
                    addPose(RedAutoPathsV2.topArtifactsPickup);
                    break;
            }
        }
    }
}
