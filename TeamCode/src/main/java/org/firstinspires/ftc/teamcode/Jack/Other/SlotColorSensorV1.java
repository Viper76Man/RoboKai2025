package org.firstinspires.ftc.teamcode.Jack.Other;

import android.graphics.ColorSpace;
import android.util.Half;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;

import java.util.Arrays;
import java.util.List;

public class SlotColorSensorV1 {
    public RevColorSensorV3 sensor;
    public void init(HardwareMap hardwareMap, String configName){
        sensor = hardwareMap.get(RevColorSensorV3.class, configName);
        sensor.enableLed(false);
    }
    public RGB getRGB() {
        return new RGB(sensor.red(), sensor.green(), sensor.blue());
    }
    public boolean isGreen(){
        return RobotConstantsV1.greenRGB.isInRange(getRGB(), 5, 5);
    }
    public boolean isPurple(){
        return RobotConstantsV1.purpleRGB.isInRange(getRGB(), 5, 5);
    }
}
