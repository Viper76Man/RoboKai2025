package org.firstinspires.ftc.teamcode.Jack.Odometry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class CustomFollower {
    public Follower follower;
    public CustomPath path = null;
    private boolean busy = false;
    public List<Integer> whichList = new ArrayList<>();

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
            switch (pathState) {
                case PATH_ONE:
                    if (!follower.isBusy()) {
                        follower.followPath(path.path);
                        busy = true;
                    } else {
                        if (follower.getCurrentTValue() >= path.tValue) {
                            if (path.pathToEnd != null) {
                                pathState = State.PATH_TWO;
                                busy = true;
                            } else {
                                path = null;
                                busy = false;
                                return;
                            }
                        }
                    }
                    break;
                case PATH_TWO:
                    if (!follower.isBusy()) {
                        follower.followPath(path.pathToEnd);
                        busy = true;
                    } else {
                        if (follower.getCurrentTValue() >= path.endTValue) {
                            path = null;
                            busy = false;
                            return;
                        }
                    }
                    break;
            }
        }

    }
    public boolean isBusy(){
        return busy;
    }
}