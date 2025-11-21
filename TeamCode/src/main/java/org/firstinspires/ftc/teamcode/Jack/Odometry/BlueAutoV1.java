package org.firstinspires.ftc.teamcode.Jack.Odometry;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group="Pedro")
public class BlueAutoV1 extends LinearOpMode {
    public BlueAutoPathsV1 paths = new BlueAutoPathsV1();

    public enum PathStates {
        START,
        TO_SHOOT_POSITION_1,
        TO_FIRST_ARTIFACTS
    }


    public ElapsedTime pathTimer = new ElapsedTime();
    public PathStates pathState = PathStates.START;
    public Follower follower;

    public void autoPathUpdate(){
        follower.update();
        switch (pathState){
            case START:
                setPathState(PathStates.TO_SHOOT_POSITION_1);
                break;
            case TO_SHOOT_POSITION_1:
                if(!follower.isBusy()){
                    follower.followPath(paths.outOfStart);
                }
                if(follower.getCurrentTValue() > paths.toShootPoseTValue){
                    setPathState(PathStates.TO_FIRST_ARTIFACTS);
                }
                break;
        }
    }


    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        paths.buildPaths();
        follower.setStartingPose(paths.startPose);
        waitForStart();
        while (opModeIsActive()) {
            autoPathUpdate();
        }
    }

    public void setPathState(PathStates pathState){
        this.pathState = pathState;
        pathTimer.reset();
    }
}