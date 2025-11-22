package org.firstinspires.ftc.teamcode.Jack.Odometry;

import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class PointRange {
    public Pose pose;
    public double minX, minY, maxX, maxY;
    public double xTolerance;
    public double yTolerance;
    public enum RangeOptions {
        X_ONLY,
        Y_ONLY,
        AT_LEAST_ONE,
        BOTH
    }
    public RangeOptions rangeOption;

    public PointRange(Pose targetPose, double xTolerance, double yTolerance, RangeOptions rangeOptions){
        this.xTolerance = xTolerance;
        this.yTolerance = yTolerance;
        this.rangeOption = rangeOptions;
        setPose(targetPose);
    }
    public PointRange(Pose targetPose, double xTolerance, double yTolerance){
        this.xTolerance = xTolerance;
        this.yTolerance = yTolerance;
        this.rangeOption = RangeOptions.BOTH;
        setPose(targetPose);
    }

    public Pose getPose() {
        return pose;
    }
    public void setPose(Pose targetPose) {
        this.pose = targetPose;
        this.minX = pose.getX() - xTolerance;
        this.minY = pose.getY() - yTolerance;
        this.maxX = pose.getX() + xTolerance;
        this.maxY = pose.getY() + yTolerance;
    }
    public void setTolerances(double xTolerance, double yTolerance){
        this.xTolerance = xTolerance;
        this.yTolerance = yTolerance;
        this.minX = pose.getX() - xTolerance;
        this.minY = pose.getY() - yTolerance;
        this.maxX = pose.getX() + xTolerance;
        this.maxY = pose.getY() + yTolerance;
    }
    public List<Double> getTolerances() {
        return Arrays.asList(xTolerance, yTolerance);
    }
    public boolean isInRange(Pose currentPose){
        boolean xInRange = minX < currentPose.getX() && currentPose.getX() < maxX;
        boolean yInRange = minY < currentPose.getY() && currentPose.getY() < maxY;
        boolean both =  (xInRange && yInRange);
        switch (rangeOption){
            case X_ONLY:
                return xInRange && !yInRange;
            case Y_ONLY:
                return !xInRange && yInRange;
            case AT_LEAST_ONE:
                return both || (xInRange || yInRange);
            case BOTH:
                return both;
        }
        return false;
    }
    public void setRangeOption(RangeOptions rangeOption){
        this.rangeOption = rangeOption;
    }
}
