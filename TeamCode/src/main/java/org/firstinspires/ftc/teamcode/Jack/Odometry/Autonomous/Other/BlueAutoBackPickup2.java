package org.firstinspires.ftc.teamcode.Jack.Odometry.Autonomous.Other;

import com.bylazar.field.Style;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Odometry.BlueAutoPathsV2;
import org.firstinspires.ftc.teamcode.Jack.Odometry.CustomFollower;
import org.firstinspires.ftc.teamcode.Jack.Other.DecodeAprilTag;
import org.firstinspires.ftc.teamcode.Jack.Other.Drawing;

import java.util.Objects;

@Disabled
public class BlueAutoBackPickup2 extends LinearOpMode {
    public CustomFollower follower;
    public BlueAutoPathsV2 pathsV2 = new BlueAutoPathsV2();
    public DecodeAprilTag obeliskTag;
    public LimelightV1 limelight = new LimelightV1();

    public boolean turnedYet = false;
    public boolean pickup1Followed = false;

    //FAKE STUFF------------------------------------------------------------------------------------
    public int ballsFired = 0;
    public ElapsedTime ballTimer = new ElapsedTime();

    public enum PathStates {
        START,
        TO_SHOOT,
        SHOOT_SET_1,
        TO_PICKUP_1,
        PICKUP_1,
        BACK_TO_SHOOT_1,
        SHOOT_SET_2,
        TO_PICKUP_2,
        PICKUP_2,
        BACK_TO_SHOOT_2,
        SHOOT_SET_3,
        IDLE
    }

    public enum ActionStates {
        DRIVE_TO_SHOOT,
        SHOOT_1,
        DRIVE_TO_BALLS_1,
        SHOOT_2,
        DRIVE_TO_BALLS_2,
        SHOOT_3,
        DRIVE_TO_BALLS_3
    }


    public PathStates pathState;
    public ActionStates actionState;

    @Override
    public void runOpMode() {
        initHardware();
        pathState = PathStates.START;
        actionState = ActionStates.DRIVE_TO_SHOOT;
        follower.setStartingPose(BlueAutoPathsV2.startPoseFar);
        limelight.startStreaming();
        while (opModeInInit()){
            obeliskTag = limelight.getLastObeliskTag();
        }
        if(obeliskTag != null) {
            telemetry.addData("Latest tag: ", obeliskTag.name());
        }
        waitForStart();
        while (opModeIsActive()){
            log();
            autoPathUpdate();
            systemStatesUpdate();
            if(RobotConstantsV1.panelsEnabled){
                draw();
            }
        }
    }

    public void initHardware(){
        follower = new CustomFollower(hardwareMap);
        limelight.init(hardwareMap);
        pathsV2.buildPaths();
    }

    public void autoPathUpdate(){
        follower.update(telemetry);
        telemetry.addData("Pose: ", follower.follower.getPose());
        switch (pathState){
            case START:
                setPathState(PathStates.TO_SHOOT);
                break;
            case TO_SHOOT:
                if(!follower.isBusy()){
                    follower.setCurrentPath(BlueAutoPathsV2.outOfStartFar);
                    setPathState(PathStates.SHOOT_SET_1);
                    break;
                }
            case SHOOT_SET_1:
                if(!follower.isBusy() && actionState != ActionStates.DRIVE_TO_BALLS_1) {
                    setActionState(ActionStates.SHOOT_1);
                }
                if(actionState == ActionStates.DRIVE_TO_BALLS_1){
                    setPathState(PathStates.TO_PICKUP_1);
                }
                break;
            case TO_PICKUP_1:
                if(!follower.isBusy()){
                    follower.setCurrentPath(BlueAutoPathsV2.toFirstArtifacts);
                    setPathState(PathStates.PICKUP_1);
                }
                break;
            case PICKUP_1:
                if(!follower.isBusy()){
                    follower.setCurrentPath(BlueAutoPathsV2.pickup1);
                    setPathState(PathStates.BACK_TO_SHOOT_1);
                }
                break;
            case BACK_TO_SHOOT_1:
                if(isLastPathName(BlueAutoPathsV2.pickup1.getName()) && !follower.isBusy()){
                    follower.setCurrentPath(BlueAutoPathsV2.overdriveBack1);
                }
                else if(isLastPathName(BlueAutoPathsV2.overdriveBack1.getName()) && follower.follower.getCurrentTValue() > BlueAutoPathsV2.backToShoot1OverdriveTValue){
                    follower.setCurrentPath(BlueAutoPathsV2.backToShoot1);
                    setPathState(PathStates.SHOOT_SET_2);
                }
                break;
            case SHOOT_SET_2:
                if(!follower.isBusy() && actionState != ActionStates.DRIVE_TO_BALLS_2) {
                    setActionState(ActionStates.SHOOT_2);
                }
                if(actionState == ActionStates.DRIVE_TO_BALLS_2){
                    setPathState(PathStates.TO_PICKUP_2);
                }
                break;
            case TO_PICKUP_2:
                if(!follower.isBusy()){
                    follower.setCurrentPath(BlueAutoPathsV2.toSecondArtifacts);
                    setPathState(PathStates.PICKUP_2);
                }
                break;
            case PICKUP_2:
                if(!follower.isBusy()){
                    follower.setCurrentPath(BlueAutoPathsV2.pickup2);
                    setPathState(PathStates.BACK_TO_SHOOT_2);
                }
                break;
            case BACK_TO_SHOOT_2:
                if(isLastPathName(BlueAutoPathsV2.pickup2.getName()) && !follower.isBusy()){
                    follower.setCurrentPath(BlueAutoPathsV2.overdriveBack1);
                }
                else if(isLastPathName(BlueAutoPathsV2.overdriveBack2.getName()) && follower.follower.getCurrentTValue() > BlueAutoPathsV2.backToShoot1OverdriveTValue){
                    follower.setCurrentPath(BlueAutoPathsV2.backToShoot2);
                    setPathState(PathStates.SHOOT_SET_3);
                }
                break;
            case SHOOT_SET_3:
                if(!follower.isBusy() && actionState != ActionStates.DRIVE_TO_BALLS_3) {
                    setActionState(ActionStates.SHOOT_3);
                }
                if(actionState == ActionStates.DRIVE_TO_BALLS_3){
                    setPathState(PathStates.IDLE);
                }
                break;
        }
    }

    public void systemStatesUpdate(){
        switch (actionState){
            case DRIVE_TO_SHOOT:
                break;
            case SHOOT_1:
                if(ballsFired < 3 && ballTimer.seconds() > 1){
                    fireBall();
                }
                if(ballsFired >= 3){
                    setActionState(ActionStates.DRIVE_TO_BALLS_1);
                    ballsFired = 3;
                }
                break;
            case SHOOT_2:
                if(ballsFired < 6 && ballTimer.seconds() > 1){
                    fireBall();
                }
                if(ballsFired >= 6){
                    setActionState(ActionStates.DRIVE_TO_BALLS_2);
                    ballsFired = 6;
                }
                break;
            case SHOOT_3:
                if(ballsFired < 9 && ballTimer.seconds() > 1){
                    fireBall();
                }
                if(ballsFired >= 9){
                    setActionState(ActionStates.DRIVE_TO_BALLS_3);
                    ballsFired = 9;
                }
                break;
        }
    }

    public void draw(){
        Drawing.drawRobot(follower.follower.getPose());
        Drawing.drawPoseHistory(follower.follower.getPoseHistory());
        if(follower.path != null) {
            Drawing.drawPath(follower.path.path, new Style("", "#3F51B5", 0.75));
            if(follower.path.pathToEnd != null){
                Drawing.drawPath(follower.path.pathToEnd, new Style("", "#3F51B5", 0.75));
            }
        }
        Drawing.sendPacket();
    }

    public void fireBall(){
        ballsFired += 1;
        ballTimer.reset();
    }

    public void setPathState(PathStates pathState){
        this.pathState = pathState;
    }
    public void setActionState(ActionStates actionState){
        this.actionState = actionState;
    }

    public void log(){
        telemetry.addLine("State: " + pathState.name());
        telemetry.addLine("Action: " + actionState.name());
        if(follower.path != null) {
            telemetry.addLine("Path: " + follower.path.getName());
        }
        telemetry.addLine("Balls fired: " + ballsFired);
        telemetry.addLine("T-value: " + follower.follower.getCurrentTValue());
        telemetry.addLine("Busy? " + follower.isBusy());
    }

    public boolean isLastPathName(String name){
        return Objects.equals(follower.lastPathName, name);
    }

}
