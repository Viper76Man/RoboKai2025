package org.firstinspires.ftc.teamcode.Jack.Other;

import java.util.Objects;

public class BallManager {
    public ArtifactSlot slot1 = new ArtifactSlot();
    public ArtifactSlot slot2 = new ArtifactSlot();
    public ArtifactSlot slot3 = new ArtifactSlot();

    public int current = 1;

    public boolean autoAdvance = true;

    public enum State {
        SHOOT,
        INTAKE
    }

    public State mode = State.INTAKE;


    public void setCurrentBall(int ball){
        current = ball;
    }

    public void next(){
        int ball = getCurrentBall() + 1;
        if(ball > 3){
            ball = 1;
        }
        setCurrentBall(ball);
    }

    public void previous(){
        int ball = getCurrentBall() - 1;
        if(ball < 1){
            ball = 3;
        }
        setCurrentBall(ball);
    }

    public void setAutoAdvance(boolean on){
        autoAdvance = on;
    }

    public int getCurrentBall(){
        return current;
    }

    public void setMode(State mode){
        this.mode = mode;
    }

    public boolean isEmpty(int ball){
        switch (ball){
            case 1:
                return Objects.equals(slot1.getColor(), ArtifactColor.NONE);
            case 2:
                return Objects.equals(slot2.getColor(), ArtifactColor.NONE);
            case 3:
                return Objects.equals(slot3.getColor(), ArtifactColor.NONE);
            default:
                return true;
        }
    }

    public void setEmpty(int ball){
        switch (ball){
            case 1:
                slot1.setColor(ArtifactColor.NONE);
                break;
            case 2:
                slot2.setColor(ArtifactColor.NONE);
                break;
            case 3:
                slot3.setColor(ArtifactColor.NONE);
                break;
        }
    }

    public void setGreen(int ball){
        switch (ball){
            case 1:
                slot1.setColor(ArtifactColor.NONE);
                break;
            case 2:
                slot2.setColor(ArtifactColor.NONE);
                break;
            case 3:
                slot3.setColor(ArtifactColor.NONE);
                break;
        }
        if(autoAdvance){
            next();
        }
    }

    public void setPurple(int ball){
        switch (ball){
            case 1:
                slot1.setColor(ArtifactColor.NONE);
                break;
            case 2:
                slot2.setColor(ArtifactColor.NONE);
                break;
            case 3:
                slot3.setColor(ArtifactColor.NONE);
                break;
        }
        if(autoAdvance){
            next();
        }
    }


    public void setBall1(ArtifactColor color){
        slot1.setColor(color);
    }

    public void setBall2(ArtifactColor color){
        slot2.setColor(color);
    }

    public void setBall3(ArtifactColor color){
        slot3.setColor(color);
    }

    public ArtifactColor getSlot1() {
        return slot1.getColor();
    }
    public ArtifactColor getSlot2() {
        return slot2.getColor();
    }
    public ArtifactColor getSlot3() {
        return slot3.getColor();
    }

}
