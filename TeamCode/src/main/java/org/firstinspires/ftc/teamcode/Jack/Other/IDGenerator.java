package org.firstinspires.ftc.teamcode.Jack.Other;

public class IDGenerator {
    public static String getRandom(int size){
        String characters = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890";
        StringBuilder sb = new StringBuilder(size);
        for(int i = 0 ; i < size; i++){
            int index = (int)(characters.length() * Math.random());
            sb.append(characters.charAt(index));
        }
        return sb.toString();
    }
}
