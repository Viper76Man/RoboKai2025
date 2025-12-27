package org.firstinspires.ftc.teamcode.Jack.Motors;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SpindexerEncoderV1 {
    public DigitalChannel channelA;
    public DigitalChannel channelB;

    public void init(HardwareMap hardwareMap){
        channelA = hardwareMap.get(DigitalChannel.class, "digital1");
        channelB = hardwareMap.get(DigitalChannel.class, "digital2");
        channelA.setMode(DigitalChannel.Mode.INPUT);
        channelB.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean getChannelAState(){
        return channelA.getState();
    }
    public boolean getChannelBState(){
        return channelB.getState();
    }

}
