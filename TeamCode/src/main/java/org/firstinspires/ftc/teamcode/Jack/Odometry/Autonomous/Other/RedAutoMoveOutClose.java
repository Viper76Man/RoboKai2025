package org.firstinspires.ftc.teamcode.Jack.Odometry.Autonomous.Other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Odometry.CustomFollower;
import org.firstinspires.ftc.teamcode.Jack.Odometry.RedAutoPathsV2;
import org.firstinspires.ftc.teamcode.Jack.Other.Drawing;

import java.util.Objects;

@Disabled
public class RedAutoMoveOutClose extends LinearOpMode {
    public CustomFollower follower;
    public RedAutoPathsV2 pathsV2 = new RedAutoPathsV2();
    public enum PathStates {
        START,
        LEAVE,
        IDLE
    }

    public enum State {
        IDLE
    }

    public State actionState = State.IDLE;

    public PathStates pathState;
    public boolean actionIsSet = false;

    @Override
    public void runOpMode() {
        initHardware();
        pathState = PathStates.START;
        follower.setStartingPose(RedAutoPathsV2.startPoseClose);
        waitForStart();
        while (opModeIsActive()) {
            log();
            autoPathUpdate();
            systemStatesUpdate();
            if (RobotConstantsV1.panelsEnabled) {
                draw();
            }
        }
    }

    public void initHardware() {
        follower = new CustomFollower(hardwareMap);
        pathsV2.buildPaths();
    }

    public void autoPathUpdate() {
        follower.update(telemetry);
        telemetry.addData("Pose: ", follower.follower.getPose());
        switch (pathState) {
            case START:
                setPathState(PathStates.LEAVE);
                break;
            case LEAVE:
                if(!follower.isBusy()){
                    follower.setCurrentPath(RedAutoPathsV2.leaveShootClose);
                }
                setPathState(PathStates.IDLE);
                break;
        }
    }

    public void systemStatesUpdate() {
    }

    public void draw() {
        Drawing.drawRobot(follower.follower.getPose());
        Drawing.drawPoseHistory(follower.follower.getPoseHistory());
        Drawing.sendPacket();
    }

    public void setPathState(PathStates pathState) {
        this.pathState = pathState;
    }

    public void setActionState(State actionState) {
        this.actionState = actionState;
        actionIsSet = false;
    }

    public void log() {
        telemetry.addLine("State: " + pathState.name());
        telemetry.addLine("Action: " + actionState.name());
    }

    public boolean isLastPathName(String name){
        return Objects.equals(follower.lastPathName, name);
    }

}
