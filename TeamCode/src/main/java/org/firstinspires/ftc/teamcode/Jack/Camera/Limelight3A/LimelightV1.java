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
import org.firstinspires.ftc.teamcode.Jack.Other.Range;

import java.nio.channels.Pipe;
import java.util.ArrayList;
import java.util.List;
import java.util.PropertyPermission;


public class LimelightV1 {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Pipeline pipeline;

    public Limelight3A limelight;

    public enum Pipeline {
        RED_GOAL,
        BLUE_GOAL,
        OBELISK
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
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
        int index = -1;
        switch (pipeline){
            case BLUE_GOAL:
                index = 1;
                break;
            case RED_GOAL:
                index = 2;
                break;
            case OBELISK:
                index = 0;
                break;
        }
        if(index != -1) {
            return limelight.pipelineSwitch(index);
        }
        else {
            return false;
        }
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
        if (!(getFiducialResults().size() <= 0)){
            return getFiducialResults().get(getFiducialResults().size() - 1);
        }
        else {
            return null;
        }
    }

    public double getTargetDistance() {
        if (getLatestAprilTagResult() != null) {
            double angleToGoalDegrees = getLatestAprilTagResult().getTargetYDegrees();
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
            double goalHeightInches = 29.5;
            //calculate distance
            return (goalHeightInches - RobotConstantsV1.LIMELIGHT_HEIGHT_FROM_GROUND_INCHES) / Math.tan(angleToGoalRadians);
        }
        return -1000000;
    }

    public boolean lockedOnTarget(Robot.Alliance alliance){
        Range redRange = new Range(RobotConstantsV1.shotAngleRedDegrees, RobotConstantsV1.degreeToleranceCamera);
        Range blueRange = new Range(RobotConstantsV1.shotAngleRedDegrees, RobotConstantsV1.degreeToleranceCamera);
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
}
