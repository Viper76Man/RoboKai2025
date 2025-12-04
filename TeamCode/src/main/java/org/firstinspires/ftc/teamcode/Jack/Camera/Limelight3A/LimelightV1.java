package org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;


public class LimelightV1 {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public Limelight3A limelight;

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

    public boolean setPipeline(int index){
        return limelight.pipelineSwitch(index);
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

    public void restart(int pipelineToSet){
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
        if (list.size() > 10) {
            list.remove(1);
        }
        return list;
    }
    public LLResultTypes.ColorResult getColorResultFromList(List<LLResultTypes.ColorResult> list, int index){
        return list.get(index);
    }

    public double getLatestAprilTagRotation(){
        return getFiducialResults().get(getFiducialResults().size() -1).getTargetPoseRobotSpace().getOrientation().getPitch();
    }

    public LLResultTypes.FiducialResult getLatestAprilTagResult(){
        return getFiducialResults().get(getFiducialResults().size() - 1);
    }
}
