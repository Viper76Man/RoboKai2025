package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.List;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class GoBildaPWMLight implements Subsystem {
    public Servo light;

    public void init(){
        light = ActiveOpMode.hardwareMap().get(Servo.class, RobotConstantsV1.pwmLightName);
        setColor(Color.YELLOW);
    }
    public enum Color {
        OFF,
        BLACK,
        RED,
        ORANGE,
        YELLOW,
        SAGE,
        GREEN,
        AZURE,
        BLUE,
        INDIGO,
        PURPLE,
        WHITE
    }

    public List<? extends Number> colorValues = Arrays.asList(0, 0.277, 0.333, 0.388, 0.444, 0.5, 0.555, 0.611, 0.667, 0.722, 1);

    public void setPWM(double pwm){
        light.setPosition(pwm);
    }

    public void setColor(Color color){
        setPWM((double) colorValues.get(color.ordinal()));
    }
}
