package org.firstinspires.ftc.teamcode.Jack.Odometry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;

@Autonomous(group = "Pedro")
public class TestAuto extends LinearOpMode {
    //VARIABLES-------------------------------------------------------------------------------------
    public ElapsedTime pathTimer = new ElapsedTime();
    public PathState pathState = PathState.START;
    public boolean pathFollowedInState = false;
    public List<Double> tValues = Arrays.asList(0.01, 0.5, 0.99);
    //FOLLOWERS/PATHING-----------------------------------------------------------------------------
    public Follower follower;
    //HARDWARE--------------------------------------------------------------------------------------

    //STATES----------------------------------------------------------------------------------------
    public enum PathState {
        START,
        OUT_OF_STARTING_ZONE,
        LEFT,
        END
    }

    //POSES-----------------------------------------------------------------------------------------
    //TODO: Update with real coordinates and make sure this works
    public Pose startPose = new Pose(0,0, Math.toRadians(90));
    public Pose outOfStartTarget = new Pose(0,100, Math.toRadians(90));
    public Pose leftTarget = new Pose(-20,50,Math.toRadians(90));
    //PATHS-----------------------------------------------------------------------------------------
    public Path outOfStart, leftPath;
    //==============================================================================================
    public void buildPaths(){
        outOfStart = new Path(new BezierLine(startPose, outOfStartTarget));
        outOfStart.setConstantHeadingInterpolation(startPose.getHeading());
        leftPath = new Path(new BezierLine(outOfStartTarget, leftTarget));
        leftPath.setConstantHeadingInterpolation(outOfStartTarget.getHeading());
    }




    //----------------------------------------------------------------------------------------------
    public void autonomousPathUpdate(){
        telemetry.addLine("Path Update");
        switch (pathState) {
            case START:
                setPathState(PathState.OUT_OF_STARTING_ZONE);
                break;
            case OUT_OF_STARTING_ZONE:
                if (!follower.isBusy()) {
                    followPath(outOfStart);
                }
                break;
            case LEFT:
                if (!follower.isBusy()) {
                    followPath(leftPath);
                }
                break;
        }
        //TODO: Add list for values of each path's t value
        if(isReadyForNextPath() && follower.isBusy() && pathTimer.seconds() > 0.5){
            if(!isAtEndOfPathStates()) {
                follower.breakFollowing();
                setPathState(getNextPathState());
            }
        }
    }






    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);

        follower.setPose(startPose);
        buildPaths();
        telemetry.addLine("Waiting for start");
        while (opModeInInit()) {
            telemetry.update();
        }
        waitForStart();
        //START-------------------------------------------------------------------------------------
        while (opModeIsActive()){
            logTelemetry();
            follower.update();
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
    public void followPath(Path path){
        follower.followPath(path);
        pathFollowedInState = true;
    }

    public boolean isReadyForNextPath(){
        return follower.isBusy() && follower.getCurrentTValue() > tValues.get(pathState.ordinal());
    }


    public void setPathState(PathState state){
        pathState = state;
        pathFollowedInState = false;
        pathTimer.reset();
    }

    public void logTelemetry(){
        telemetry.addData("Path State: ", pathState.name());
        telemetry.addData("Position: ", follower.getPose().toString());
        telemetry.addData("Current T-value: ", follower.getCurrentTValue());
        telemetry.update();
    }
}
