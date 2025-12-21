package org.firstinspires.ftc.teamcode.Jack.Odometry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Other.Range;

public class CustomFollower {
    public Follower follower;
    public CustomPath path = null;
    private boolean busy = false;
    private boolean turning = false;
    public String lastPathName = "";

    private Range headingRange;

    public enum State {
        PATH_ONE,
        PATH_TWO
    }

    public State pathState;

    public CustomFollower(Follower follower) {
        this.follower = follower;
    }

    public CustomFollower(HardwareMap hardwareMap) {
        this.follower = Constants.createFollower(hardwareMap);
    }

    public void setCurrentPath(CustomPath path) {
        //TODO: Clear queue?
        this.path = path;
        this.pathState = State.PATH_ONE;
        this.lastPathName = path.getName();
        follower.breakFollowing();
        this.busy = true;
    }

    public void setStartingPose(Pose pose) {
        follower.setStartingPose(pose);
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public void update(Telemetry telemetry) {
        telemetry.update();
        follower.update();
        if (path != null){
            turning = false;
            switch (pathState) {
                case PATH_ONE:
                    if (!follower.isBusy()) {
                        follower.followPath(path.path);
                        busy = true;
                    } else {
                        if (follower.getCurrentTValue() >= path.tValue) {
                            follower.breakFollowing();
                            if (path.pathToEnd != null) {
                                pathState = State.PATH_TWO;
                                follower.breakFollowing();
                                busy = true;
                            } else {
                                follower.breakFollowing();
                                path = null;
                                busy = false;
                            }
                        }
                    }
                    break;
                case PATH_TWO:
                    if (!follower.isBusy()) {
                        follower.breakFollowing();
                        follower.followPath(path.pathToEnd);
                        busy = true;
                    } else {
                        if (follower.getCurrentTValue() >= path.endTValue) {
                            follower.breakFollowing();
                            path = null;
                            busy = false;
                            pathState = State.PATH_ONE;
                            return;
                        }
                    }
                    break;
            }
        } else if(turning) {
            if(headingRange.isInRange(Math.toDegrees(follower.getHeading()))) {
                turning = false;
                busy = false;
            }
        }

    }

    public void turnTo(double headingRad){
        follower.turnTo(headingRad);
        headingRange = new Range(Math.toDegrees(headingRad), 3);
        busy = true;
        turning = true;
    }
    public boolean isBusy(){
        return busy;
    }
}