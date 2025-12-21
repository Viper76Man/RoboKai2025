package org.firstinspires.ftc.teamcode.Jack.Odometry;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Jack.Other.IDGenerator;

public class CustomPath {
    public double tValue;
    public Path path;
    public Path pathToEnd = null;
    public double endTValue;
    private boolean multiplePaths = false;
    public String name = "";
    public String id = "";
    public CustomPath(Path path, double minTValue){
        this.path = path;
        this.tValue = minTValue;
        this.id = generateID();
    }

    public CustomPath(Pose startPoint, Pose endPoint, double minTValue){
        this.path = new Path(new BezierLine(startPoint, endPoint));
        this.tValue = minTValue;
        this.id = generateID();
    }

    public CustomPath(Pose startPoint, Pose endPoint, Pose overdrivePoint, double minTValue, double endTValue) {
        this.path = new Path(new BezierLine(startPoint, overdrivePoint));
        this.pathToEnd = new Path(new BezierLine(overdrivePoint, endPoint));
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

    public void setLinearHeadingInterpolationPath1(double start, double end){
        this.path.setLinearHeadingInterpolation(start, end);
    }

    public void setLinearHeadingInterpolationPath2(double start, double end){
        this.pathToEnd.setLinearHeadingInterpolation(start, end);
    }

    public void setConstantHeadingInterpolationPath1(double headingRadians){
        this.path.setConstantHeadingInterpolation(headingRadians);
    }

    public void setConstantHeadingInterpolationPath2(double headingRadians){
        this.pathToEnd.setConstantHeadingInterpolation(headingRadians);
    }

    public void setName(String name){
        this.name = name;
    }

    public String generateID(){
        return IDGenerator.getRandom(20);
    }
    public String getName(){
        return this.name;
    }
}
