package org.firstinspires.ftc.teamcode.Jack.Other;

public class TagIDToAprilTag {
    public DecodeAprilTag getTag(int id){
        switch (id){
            case 20:
                return DecodeAprilTag.BLUE_GOAL;
            case 21:
                return DecodeAprilTag.OBELISK_GPP;
            case 22:
                return DecodeAprilTag.OBELISK_PGP;
            case 23:
                return DecodeAprilTag.OBELISK_PPG;
            case 24:
                return DecodeAprilTag.RED_GOAL;
            default:
                return DecodeAprilTag.INVALID;
        }
    }
}
