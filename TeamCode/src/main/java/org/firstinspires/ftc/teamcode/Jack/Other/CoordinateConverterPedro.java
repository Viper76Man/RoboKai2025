package org.firstinspires.ftc.teamcode.Jack.Other;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class CoordinateConverterPedro {
    public Pose toPose(Pose3D pose3D){
        return new Pose(pose3D.getPosition().x, pose3D.getPosition().y, pose3D.getOrientation().getPitch()).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }

    public Pose toStandard(Pose pose){
        return pose.getAsCoordinateSystem(FTCCoordinates.INSTANCE);
    }

    public Pose toPedro(Pose pose){
        return pose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
}
