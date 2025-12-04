package org.firstinspires.ftc.teamcode.Jack.Other;

import com.pedropathing.geometry.Pose;

public class Range {
    private enum RangeMode{
        POSE,
        DOUBLE
    }

    private final RangeMode mode;
    private double left, right, target;
    private double xLeft, xRight, xTarget, yLeft, yRight, yTarget;
    private boolean error;
    public Range(Pose target, double xTolerance, double yTolerance){
        yTolerance = Math.abs(yTolerance);
        xTolerance = Math.abs(xTolerance);
        this.xLeft = target.getX() - xTolerance;
        this.xRight = target.getX() + xTolerance;
        this.xTarget= target.getX();
        this.yLeft = target.getY() - yTolerance;
        this.yTarget = target.getY();
        this.yRight = target.getY() + yTolerance;
        mode = RangeMode.POSE;
    }

    public Range(Pose target, double xLeft, double yLeft, double xRight, double yRight){
        this.xTarget = target.getX();
        this.yTarget = target.getY();
        this.xLeft = xLeft;
        this.xRight = xRight;
        this.yLeft = yLeft;
        this.yRight = yRight;
        mode = RangeMode.POSE;
    }
    public Range(double left, double center, double right){
        this.left = left;
        this.target = center;
        this.right = right;
        mode = RangeMode.DOUBLE;
    }

    public Range(double target, double tolerance){
        this.target = target;
        this.left = target - tolerance;
        this.right = target + tolerance;
        mode = RangeMode.DOUBLE;
    }

    public boolean isInRange(Pose pose){
        if(mode !=RangeMode.POSE){
            return false;
        }
        else {
            return pose.getX() > xLeft && pose.getX() < xRight && pose.getY() > yLeft && pose.getY() < yRight;
        }
    }

    public boolean isInRange(double num){
        if(mode !=RangeMode.DOUBLE){
            return false;
        }
        else {
            return num > left && num < right;
        }
    }

}
