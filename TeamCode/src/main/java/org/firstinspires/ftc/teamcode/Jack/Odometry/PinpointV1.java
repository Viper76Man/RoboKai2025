package org.firstinspires.ftc.teamcode.Jack.Odometry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;

public class PinpointV1 {
    public GoBildaPinpointDriver pinpoint;
    public void init(HardwareMap hardwareMap){
        this.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, RobotConstantsV1.pinpointName);
        this.pinpoint.setOffsets(RobotConstantsV1.strafePodX, RobotConstantsV1.forwardPodY, DistanceUnit.INCH);
    }
    public void update(){
        pinpoint.update();
    }
    public void resetPosAndIMU(){
        pinpoint.resetPosAndIMU();
    }

    public GoBildaPinpointDriver.DeviceStatus getStatus(){
        return pinpoint.getDeviceStatus();
    }

    public boolean isReady(){
        return getStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }
}
