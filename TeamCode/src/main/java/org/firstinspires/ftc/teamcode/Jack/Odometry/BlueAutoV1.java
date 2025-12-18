package org.firstinspires.ftc.teamcode.Jack.Odometry;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Jack.Drive.GamepadV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.Robot;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotV2;
import org.firstinspires.ftc.teamcode.Jack.Other.DecodeAprilTag;
import org.firstinspires.ftc.teamcode.Jack.Other.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.Jack.Other.TagIDToAprilTag;

@Disabled
public class BlueAutoV1 extends LinearOpMode {
    public BlueAutoPathsV1 paths = new BlueAutoPathsV1();
    public RobotV2 robot = new RobotV2();
    public TagIDToAprilTag tagConverter = new TagIDToAprilTag();

    public enum PathStates {
        START,
        TO_SHOOT_POSITION_1,
        TO_FIRST_ARTIFACTS
    }

    public enum AutoPath {
        FRONT_FIRST,
        MIDDLE_FIRST,
        BACK_FIRST
    }


    public ElapsedTime pathTimer = new ElapsedTime();
    public ElapsedTime cameraTimer = new ElapsedTime();
    public PathStates pathState = PathStates.START;
    public Follower follower;
    public AutoPath path;

    public void autoPathUpdate(){
        telemetry.addData("Detected path: ", path.name());
        follower.update();
        switch (pathState){
            case START:
                setPathState(PathStates.TO_SHOOT_POSITION_1);
                break;
            case TO_SHOOT_POSITION_1:
                if(!follower.isBusy()){
                    follower.followPath(paths.outOfStart);
                }
                if(follower.getCurrentTValue() > BlueAutoPathsV1.toShootPoseTValue){
                    setPathState(PathStates.TO_FIRST_ARTIFACTS);
                }
                break;
        }
    }

    public void initUpdate(){
        if(robot.isObeliskResultValid() && robot.limelight.getLatestAprilTagResult() != null){
           DecodeAprilTag latestTag = tagConverter.getTag(robot.limelight.getLatestAprilTagResult().getFiducialId());
           if(latestTag == DecodeAprilTag.OBELISK_PPG){
               path = AutoPath.FRONT_FIRST;
           }
           else if(latestTag == DecodeAprilTag.OBELISK_PGP){
               path = AutoPath.MIDDLE_FIRST;
           }
           else {
               path = AutoPath.BACK_FIRST;
           }
        }
        else if(cameraTimer.seconds() > 4){
            path = AutoPath.FRONT_FIRST;
        }
    }


    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        GamepadV1 gamepadv1 = new GamepadV1();
        gamepadv1.init(gamepad1, 0.3);
        robot.init(RobotV2.Mode.AUTONOMOUS, Robot.Alliance.BLUE, hardwareMap, gamepadv1, telemetry);
        paths.buildPaths();
        follower.setStartingPose(BlueAutoPathsV1.startPose);
        while (opModeInInit() && !isStopRequested()){
            initUpdate();
        }
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            autoPathUpdate();
        }
    }

    public void setPathState(PathStates pathState){
        this.pathState = pathState;
        pathTimer.reset();
    }
}