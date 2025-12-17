package org.firstinspires.ftc.teamcode.Jack.Odometry;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Jack.Other.CustomPath;

import java.util.ArrayList;
import java.util.List;

public class CustomFollower {
    public Follower follower;
    public List<CustomPath> queue = new ArrayList<>();
    public List<Path> pathQueue = new ArrayList<>();


    public CustomPath path = null;
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

    public void addToQueue(CustomPath path) {
        //TODO: Clear queue?
        queue.add(path);
    }

    public void update() {
        if (path == null && !queue.isEmpty()) {
            path = queue.get(0);
            queue.remove(0);
            pathState = State.PATH_ONE;
        }
        switch (pathState) {
            case PATH_ONE:
                if (!follower.isBusy()) {
                    follower.followPath(path.path);
                } else {
                    if (follower.getCurrentTValue() >= path.tValue) {
                        if (path.pathToEnd != null) {
                            pathState = State.PATH_TWO;
                        } else {
                            path = getNextQueuePath();
                        }
                    }
                }
                break;
            case PATH_TWO:
                if (!follower.isBusy()) {
                    follower.followPath(path.pathToEnd);
                } else {
                    if (follower.getCurrentTValue() >= path.endTValue) {
                        path = getNextQueuePath();
                    }
                }
                break;
        }

    }

    public CustomPath getNextQueuePath() {
        return queue.get(0);
    }
}
