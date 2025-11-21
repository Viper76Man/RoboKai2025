package org.firstinspires.ftc.teamcode.Jack.Servos;

import org.firstinspires.ftc.teamcode.Jack.Other.ArtifactColor;
import org.firstinspires.ftc.teamcode.Jack.Other.ArtifactSlot;

public class ArtifactSorterV1 {
    public ArtifactColor colorCache;
    public ArtifactSlot slot1 = new ArtifactSlot();
    public ArtifactSlot slot2 = new ArtifactSlot();
    public ArtifactSlot slot3 = new ArtifactSlot();

    public void previous(){
        colorCache = slot3.getColor();
        slot3.setColor(slot2.getColor());
        slot2.setColor(slot1.getColor());
        slot1.setColor(colorCache);
    }
    public void next(){
        colorCache = slot1.getColor();
        setSlotColor(1, slot2.getColor());
        setSlotColor(2, slot3.getColor());
        setSlotColor(3, colorCache);
    }
    public void setSlotColor(int slot, ArtifactColor color){
        switch (slot){
            case 1:
                slot1.setColor(color);
                break;
            case 2:
                slot2.setColor(color);
                break;
            case 3:
                slot3.setColor(color);
                break;
        }
    }

}
