package org.firstinspires.ftc.teamcode.Jack.Other;

import android.graphics.ColorSpace;
import android.util.Half;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV1;

import java.util.Arrays;
import java.util.List;

public class SlotColorSensorV1 {
    public RevColorSensorV3 sensor;
    public int loops = 0;
    public double avgGreen = 0;
    public double dist = 0;
    public double green = 0;
    public double lastGreen = 0;
    public ElapsedTime captureTimer = new ElapsedTime();
    public ArtifactColor current = ArtifactColor.NONE;

    public void init(HardwareMap hardwareMap, String configName){
        sensor = hardwareMap.get(RevColorSensorV3.class, configName);
        sensor.enableLed(true);
    }
    public RGB getRGB() {
        return new RGB(sensor.red(), sensor.green(), sensor.blue());
    }
    public NormalizedRGBA getNormalizedRGB() {
        return sensor.getNormalizedColors();
    }

    public void update(SpindexerMotorV1.State state, boolean spindexerReady) {
        dist = sensor.getDistance(DistanceUnit.MM);
        if (loops > 0) {
            avgGreen = green / loops;
        } else {
            avgGreen = 0;
        }

        if (avgGreen < RobotConstantsV1.MIN_G_VALUE_COLOR_SENSOR && loops >= 5) {
            switch (state) {
                case BALL_1_INTAKE:
                case BALL_2_INTAKE:
                case BALL_3_INTAKE:
                    loops = 0;
                    green = 0;
                    avgGreen = 0;
                    current = ArtifactColor.PURPLE;
            }
        } else if (avgGreen > RobotConstantsV1.MIN_G_VALUE_COLOR_SENSOR && loops >= 5) {
            switch (state) {
                case BALL_1_INTAKE:
                case BALL_2_INTAKE:
                case BALL_3_INTAKE:
                    loops = 0;
                    green = 0;
                    avgGreen = 0;
                    current = ArtifactColor.PURPLE;
            }
        }
        //change 26 to 10
        else if (loops < 5 && captureTimer.seconds() > 0.03 && spindexerReady && RobotConstantsV1.MAX_DISTANCE_COLOR_SENSOR > dist && dist > 10) {
            lastGreen = sensor.green();
            double brightness = sensor.red() +
                    lastGreen +
                    sensor.blue();
            if (brightness > 185) {
                green = green + lastGreen;
                lastGreen = 0;
                loops = loops + 1;
                captureTimer.reset();
            }
        }
        if(dist > RobotConstantsV1.MAX_DISTANCE_COLOR_SENSOR){
            current = ArtifactColor.NONE;
        }
    }

    public ArtifactColor getCurrent(){
        return this.current;
    }

    public void clear(){
        this.current = ArtifactColor.NONE;
        loops = 0;
        green = 0;
        avgGreen = 0;
    }

    public boolean isGreen(){
        return RobotConstantsV1.greenRGB.isInRange(getRGB(), 5, 5);
    }
    public boolean isPurple(){
        return RobotConstantsV1.purpleRGB.isInRange(getRGB(), 5, 5);
    }

    public void log(TelemetryManager telemetryM, Telemetry telemetry){
        telemetryM.addLine("Has ball? " + (RobotConstantsV1.MAX_DISTANCE_COLOR_SENSOR > dist));
        telemetryM.addLine("Distance: " + dist);
        telemetryM.addLine("Is not too close?: " + (dist > 26));
        telemetryM.addLine("Green captures: " + loops);
        telemetryM.addLine("Total green: " + green);
        telemetryM.addLine("Avg green: " + avgGreen);
        telemetryM.addLine("\n");
        telemetryM.addLine("Red: " + sensor.red());
        telemetryM.addLine("Green: "+ sensor.green());
        telemetryM.addLine("Blue: "+ sensor.blue());
        telemetryM.addLine("\n");
    }
}
