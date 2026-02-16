package org.firstinspires.ftc.teamcode.Jack.Other;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;

import java.util.List;

public class Sensors {
    public List<LynxModule> hubs;
    public int arcMotorPos = 0;

    public DcMotor arcMotor;



    public double spindexerPos = 0;
    public DcMotor spindexer;




    public void init(HardwareMap hardwareMap){
        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub: hubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        arcMotor = hardwareMap.get(DcMotor.class, RobotConstantsV1.arcShooterName);
        spindexer = hardwareMap.get(DcMotor.class, RobotConstantsV1.spindexerMotorName);
    }


    public void update(){
        arcMotorPos = arcMotor.getCurrentPosition();
        spindexerPos = spindexer.getCurrentPosition();
        for(LynxModule hub: hubs){
            hub.clearBulkCache();
        }
    }
}
