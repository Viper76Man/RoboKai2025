package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV1;
import org.firstinspires.ftc.teamcode.Jack.Other.BallManager;
import org.firstinspires.ftc.teamcode.Jack.Other.SlotColorSensorV1;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;

public class ColorSensorV3 implements Subsystem {
    public SlotColorSensorV1 sensor = new SlotColorSensorV1();
    public SpindexerMotorV1 spindexer = new SpindexerMotorV1();
    public BallManager manager;
    public void init(HardwareMap hardwareMap, BallManager manager){
        this.manager = manager;
        sensor.init(hardwareMap, RobotConstantsV1.colorSensor1);
        sensor.sensor.setGain(10);
        spindexer.init(hardwareMap);
    }

    public ColorSensorUpdate update(){
        return new ColorSensorUpdate();
    }

    public class ColorSensorUpdate extends Command {
        @Override
        public void update() {
            sensor.update(spindexer.state, spindexer.isSpindexerReady());
            if ((sensor.isPurple() || sensor.isGreen()) && sensor.getNormalizedRGB().green >= 0.03) {
                manager.next();
                sensor.clear();
            }
        }

        @Override
        public boolean isDone() {
            return false;
        }
    }
}