package org.firstinspires.ftc.teamcode.Jack.Odometry;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class TestAuto extends LinearOpMode {
    //VARIABLES-------------------------------------------------------------------------------------
    public ElapsedTime pathTimer = new ElapsedTime();
    public PathState pathState = PathState.START;
    //FOLLOWERS/PATHING-----------------------------------------------------------------------------
    public Follower follower;

    //HARDWARE--------------------------------------------------------------------------------------
    public PinpointV1 pinpoint = new PinpointV1();
    public Drivetrain drivetrain;

    //STATES----------------------------------------------------------------------------------------
    public enum PathState {
        START,
        OUT_OF_STARTING_ZONE,
        END
    }


    //POSES-----------------------------------------------------------------------------------------
    //TODO: Update with real coordinates and make sure this works
    public Pose startPose = new Pose(0,0, Math.toRadians(0));
    public Pose outOfStartTarget = new Pose(0,10, Math.toRadians(0));
    //PATHS-----------------------------------------------------------------------------------------
    public Path outOfStart;

    //----------------------------------------------------------------------------------------------

    public void buildPaths(){
        outOfStart = new Path(new BezierLine(startPose, outOfStartTarget));
    }




    //----------------------------------------------------------------------------------------------
    public void autonomousPathUpdate(){
        switch (pathState){
            case START:
                setPathState(PathState.OUT_OF_STARTING_ZONE);
                break;
            case OUT_OF_STARTING_ZONE:
                //TODO: Look at gyro meeting notes and make sure this works
                if(!follower.isBusy()){
                    follower.followPath(outOfStart);
                }
                break;
        }
        //TODO: Add list for values of each path's t value
        if(follower.getCurrentTValue() > 0.8){
            if(!isAtEndOfPathStates()) {
                setPathState(getNextPathState());
            }
        }
    }






    public void setPathState(PathState state){
        pathState = state;
        pathTimer.reset();
    }


    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        pinpoint.resetPosAndIMU();
        while (!pinpoint.isReady()) {
            idle();
        }
        follower.setPose(startPose);
        buildPaths();
        waitForStart();
        //START-------------------------------------------------------------------------------------
        while (opModeIsActive()){
            pinpoint.update();
            autonomousPathUpdate();
        }
    }

    //FUNCTIONS-------------------------------------------------------------------------------------
    public boolean isAtEndOfPathStates(){
        return PathState.valueOf(pathState.name()).ordinal() + 1 == PathState.values().length;
    }
    public PathState getNextPathState(){
        return PathState.values()[PathState.valueOf(pathState.name()).ordinal() + 1];
    }
}
