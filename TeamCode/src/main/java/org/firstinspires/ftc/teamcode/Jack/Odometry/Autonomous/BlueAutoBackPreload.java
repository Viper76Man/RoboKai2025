package org.firstinspires.ftc.teamcode.Jack.Odometry.Autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Odometry.BlueAutoPathsV2;
import org.firstinspires.ftc.teamcode.Jack.Odometry.CustomFollower;
import org.firstinspires.ftc.teamcode.Jack.Odometry.DecodeFieldLocalizer;
import org.firstinspires.ftc.teamcode.Jack.Other.DecodeAprilTag;
import org.firstinspires.ftc.teamcode.Jack.Other.Drawing;

@Autonomous
public class BlueAutoBackPreload extends LinearOpMode {
    public CustomFollower follower;
    public BlueAutoPathsV2 pathsV2 = new BlueAutoPathsV2();
    public DecodeAprilTag obeliskTag;
    public LimelightV1 limelight = new LimelightV1();
    //FAKE STUFF------------------------------------------------------------------------------------
    public int ballsFired = 0;
    public ElapsedTime ballTimer = new ElapsedTime();

    public enum PathStates {
        START,
        TO_SHOOT,
        SHOOT_SET_1,
        TO_PICKUP_1,
        TURN_TO_PICKUP_1,
        PICKUP_1,
        BACK_TO_SHOOT_1,
        SHOOT_SET_2
    }

    public enum ActionStates {
        DRIVE_TO_SHOOT,
        SHOOT_1,
        DRIVE_TO_BALLS_1
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
                break;
            case SHOOT_SET_1:
                if(!follower.isBusy() && actionState != ActionStates.DRIVE_TO_BALLS_1) {
                    setActionState(ActionStates.SHOOT_1);
                }
                if(actionState == ActionStates.DRIVE_TO_BALLS_1){
                    setPathState(PathStates.TO_PICKUP_1);
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
                }
                break;
        }
    }

    public void draw(){
        Drawing.drawRobot(follower.follower.getPose());
        Drawing.drawPoseHistory(follower.follower.getPoseHistory());
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
        telemetry.addLine("Balls fired: " + ballsFired);
        telemetry.addLine("T-value: " + follower.follower.getCurrentTValue());
        telemetry.addLine("Busy? " + follower.isBusy());
    }

}
