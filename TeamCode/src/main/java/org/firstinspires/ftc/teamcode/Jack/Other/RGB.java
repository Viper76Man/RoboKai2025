package org.firstinspires.ftc.teamcode.Jack.Other;

import androidx.annotation.NonNull;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.List;

public class RGB {
    public int r, g, b;
    public RGB(int red, int green, int blue){
        this.r = red;
        this.g = green;
        this.b = blue;
    }
    public RGB(float red, float green, float blue){
        this.r = (int)red;
        this.g = (int)green;
        this.b = (int)blue;
    }

    public int getRed(){
        return r;
    }
    public int getGreen(){
        return g;
    }
    public int getBlue(){
        return b;
    }
    public List<Integer> getRGB(){
        return Arrays.asList(r,g,b);
    }
    public void setRGB(int red, int green, int blue){
        this.r = red;
        this.g = green;
        this.b = blue;
    }

    public boolean isInRange(RGB rgb, int lowerTolerance, int higherTolerance){
        return isInRedRange(rgb.getRed(), lowerTolerance, higherTolerance) && isInGreenRange(rgb.getGreen(), lowerTolerance, higherTolerance) && isInBlueRange(rgb.getBlue(), lowerTolerance, higherTolerance);
    }

    public boolean isInRedRange(int red, int redLowerTolerance, int redHigherTolerance){
        return (r - Math.abs(redLowerTolerance)) < red && red < (r + Math.abs(redHigherTolerance));
    }
    public boolean isInGreenRange(int green, int greenLowerTolerance, int greenHigherTolerance){
        return (g - Math.abs(greenLowerTolerance)) < green && green < (g + Math.abs(greenHigherTolerance));
    }
    public boolean isInBlueRange(int blue, int blueLowerTolerance, int blueHigherTolerance){
        return (b - Math.abs(blueLowerTolerance)) < blue && blue < (b + Math.abs(blueHigherTolerance));
    }

    public String asText(){
        return r + ", " + g + ", "+ b;
    }
}
