package org.firstinspires.ftc.teamcode.Jack.Other;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import java.util.Random;

public class CustomPath {
    public double tValue;
    public Path path;
    public Path pathToEnd = null;
    public double endTValue;
    private boolean multiplePaths = false;
    public String id = "";
    public CustomPath(Path path, double minTValue){
        this.path = path;
        this.tValue = minTValue;
        this.id = generateID();
    }

    public CustomPath(Pose startPoint, Pose endPoint, double minTValue){
        this.path = new Path(new BezierLine(startPoint, endPoint));
        this.path.setLinearHeadingInterpolation(startPoint.getHeading(), endPoint.getHeading());
        this.tValue = minTValue;
        this.id = generateID();
    }

    public CustomPath(Pose startPoint, Pose endPoint, Pose overdrivePoint, double minTValue, double endTValue) {
        this.path = new Path(new BezierLine(startPoint, overdrivePoint));
        this.path.setLinearHeadingInterpolation(startPoint.getHeading(), overdrivePoint.getHeading());
        this.pathToEnd = new Path(new BezierLine(overdrivePoint, endPoint));
        this.pathToEnd.setLinearHeadingInterpolation(overdrivePoint.getHeading(), endPoint.getHeading());
        this.tValue = minTValue;
        this.endTValue = endTValue;
        this.multiplePaths = true;
        this.id = generateID();
    }

    public double getMinTValue(){
        return tValue;
    }

    public double geEndTValue(){
        if(multiplePaths) {
            return endTValue;
        }
        else {
            return -1;
        }
    }

    public String generateID(){
        return IDGenerator.getRandom(20);
    }
}
