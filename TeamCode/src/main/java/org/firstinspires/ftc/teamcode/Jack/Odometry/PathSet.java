package org.firstinspires.ftc.teamcode.Jack.Odometry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

public class PathSet {
    public List<Path> pathsList;
    public Follower follower;
    public Path current;
    public ElapsedTime pathTimer = new ElapsedTime();
    public PathSet(Follower follower, List<Path> pathsList){
        this.pathsList = pathsList;
        this.follower = follower;
    }
    public void followPath(Path path){
        follower.followPath(path);
        resetPathTimer();
        this.current = path;
    }

    public void followPath(int listIndex){
        try {
            follower.followPath(pathsList.get(listIndex));
            this.current = pathsList.get(listIndex);
            resetPathTimer();
        }
        catch (Exception e){
            return;
        }

    }

    public void followNext(){
        followPath(getNextPath(current));
    }

    public boolean isReadyForNext(){
        Path nextPath = getNextPath(current);
        Pose poseA = current.getPose(0);
        Pose poseB = current.getPose(1);
        Pose poseC = nextPath.getPose(0);

        return !follower.isBusy();
    }

    public Path getNextPath(Path path){
        return pathsList.get(pathsList.indexOf(path) + 1);
    }
    public double getPathTimerSeconds(){
        return pathTimer.seconds();
    }

    public void resetPathTimer(){
        pathTimer.reset();
    }

    public Direction getDirection(Pose a, Pose b){
        return Direction.UP;
    }
}
