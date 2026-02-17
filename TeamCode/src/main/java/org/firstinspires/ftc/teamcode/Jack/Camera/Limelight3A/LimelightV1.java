package org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A;

import com.bylazar.limelightproxy.LimelightProxyConfig;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Jack.Drive.Robot;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Other.DecodeAprilTag;
import org.firstinspires.ftc.teamcode.Jack.Other.Range;
import org.firstinspires.ftc.teamcode.Jack.Other.TagIDToAprilTag;

import java.nio.channels.Pipe;
import java.util.ArrayList;
import java.util.List;
import java.util.PropertyPermission;


public class LimelightV1 {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Pipeline pipeline;

    public Limelight3A limelight;

    public int pipe = -1;

    public int BLUE_GOAL = 1;
    public int OBELISK = 0;
    public int RED_GOAL = 2;

    public enum Pipeline {
        RED_GOAL,
        BLUE_GOAL,
        OBELISK,
        ERROR
    }

    public void init(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.limelight = this.hardwareMap.get(Limelight3A.class, "limelight");
    }

    public Limelight3A getLimelight(){
        return limelight;
    }

    public int getVersion(){
        return limelight.getVersion();
    }

    public void startStreaming(){
        limelight.start();
    }

    public void close(){
        limelight.close();
    }

    public boolean isConnected(){
        return limelight.isConnected();
    }

    public boolean isRunning(){
        return limelight.isRunning();
    }

    public boolean setPipeline(Pipeline pipeline){
        switch (pipeline){
            case BLUE_GOAL:
                pipe = BLUE_GOAL;
                break;
            case RED_GOAL:
                pipe = RED_GOAL;
                break;
            case OBELISK:
                pipe = OBELISK;
                break;
        }
        if(pipe != -1) {
            return limelight.pipelineSwitch(pipe);
        }
        else {
            return false;
        }
    }

    public Pipeline getPipeline() {
        if (pipe == BLUE_GOAL) {
            return Pipeline.BLUE_GOAL;
        }
        else if(pipe == RED_GOAL){
            return Pipeline.RED_GOAL;
        }
        else if(pipe == OBELISK){
            return Pipeline.OBELISK;
        }
        return Pipeline.ERROR;
    }

    public Pipeline getPipelineFromID(int id) {
        if (id == BLUE_GOAL) {
            return Pipeline.BLUE_GOAL;
        }
        else if(id == RED_GOAL){
            return Pipeline.RED_GOAL;
        }
        else if(id == OBELISK){
            return Pipeline.OBELISK;
        }
        return Pipeline.ERROR;
    }

    public LLResult getLatestResult(){
        if(limelight.getLatestResult() != null) {
            return limelight.getLatestResult();
        }
        else {
            return null;
        }
    }

    public LLStatus getStatus(){
        return limelight.getStatus();
    }

    public boolean captureSnapshot(String snapshotName){
        return limelight.captureSnapshot(snapshotName);
    }

    public boolean deleteSnapshot(String snapshotName){
        return limelight.deleteSnapshot(snapshotName);
    }

    public void restart(Pipeline pipelineToSet){
        limelight.close();
        startStreaming();
        setPipeline(pipelineToSet);
    }

    public double getFps(){
        return getStatus().getFps();
    }
    public void pause(){
        limelight.pause();
    }
    public List<LLResultTypes.ColorResult> getColorResults(){
        List<LLResultTypes.ColorResult> list = new ArrayList<>();
        if (limelight.getLatestResult() != null) {
            return limelight.getLatestResult().getColorResults();
        }
        else {
            return list;
        }
    }

    public List<LLResultTypes.FiducialResult> getFiducialResults(){
        List<LLResultTypes.FiducialResult> list = new ArrayList<>();
        if (limelight.getLatestResult() != null) {
            list = limelight.getLatestResult().getFiducialResults();
        }
        return list;
    }
    public LLResultTypes.ColorResult getColorResultFromList(List<LLResultTypes.ColorResult> list, int index){
        return list.get(index);
    }

    public double getLatestAprilTagRotation(){
        if(getLatestAprilTagResult() == null){
            return 9999;
        }
        return getLatestAprilTagResult().getTargetPoseRobotSpace().getOrientation().getPitch(AngleUnit.DEGREES);
    }

    public double getLatestBotRotation(){
        if(getLatestAprilTagResult() == null){
            return 9999;
        }
        return getLatestAprilTagResult().getTargetPoseRobotSpace().getOrientation().getPitch(AngleUnit.DEGREES);
    }

    public LLResultTypes.FiducialResult getLatestAprilTagResult(){
        List<LLResultTypes.FiducialResult> results = getFiducialResults();
        if (!results.isEmpty()){
            return results.get(results.size()  - 1);
        }
        else {
            return null;
        }
    }

    public double getTargetDistance() {
        LLResultTypes.FiducialResult result = getLatestAprilTagResult();
        if (result != null) {
            double angleToGoalDegrees = result.getTargetXDegrees();
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
            double goalHeightInches = 29.5;
            //calculate distance
            return Math.abs((goalHeightInches - RobotConstantsV1.LIMELIGHT_HEIGHT_FROM_GROUND_INCHES) / Math.tan(angleToGoalRadians));
        }
        return 0;
    }

    public boolean lockedOnTarget(Robot.Alliance alliance){
        Range redRange = new Range(RobotConstantsV1.shotAngleRedDegrees, RobotConstantsV1.degreeToleranceCamera);
        Range blueRange = new Range(RobotConstantsV1.shotAngleBlueDegrees, RobotConstantsV1.degreeToleranceCamera);
        if(getLatestAprilTagResult() != null) {
            switch (alliance) {
                case RED:
                    return redRange.isInRange(getLatestAprilTagResult().getTargetXDegrees());
                case BLUE:
                    return blueRange.isInRange(getLatestAprilTagResult().getTargetXDegrees());
            }
        }
        else {
            return false;
        }
        return false;
    }
    public DecodeAprilTag getLastObeliskTag(){
        TagIDToAprilTag converter = new TagIDToAprilTag();
        LLResultTypes.FiducialResult result = getLatestAprilTagResult();
        Pipeline pipeline_ = getPipeline();
        if(pipeline_ != Pipeline.OBELISK){
            setPipeline(Pipeline.OBELISK);
        }
        if(result != null) {
            int id = result.getFiducialId();
            return converter.getTag(id);
        }
        return null;
    }
}
