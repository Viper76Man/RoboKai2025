package org.firstinspires.ftc.teamcode.Jack.Odometry;

import com.bylazar.field.PanelsField;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;

public class DecodeFieldLocalizer {
    public Pose blueGoalCenter = new Pose(13,134.7, Math.toRadians(110));
    public Pose redGoalCenter = new Pose(131, 134.7, Math.toRadians(70));
    public Pose launchZoneCenter = new Pose(72, 12);
    public double getDistanceFromRedGoal(Pose pose){
        return pose.distanceFrom(redGoalCenter);
    }
    public double getDistanceFromBlueGoal(Pose pose){
        return pose.distanceFrom(blueGoalCenter);
    }

    public double getDistanceFromLaunchZone(Pose pose){
        return pose.distanceFrom(launchZoneCenter);
    }
    public boolean isRobotInBackLaunchZone(Pose pose){
        return pose.distanceFrom(launchZoneCenter) <= RobotConstantsV1.maxLaunchZoneDistance;
    }
    public double getHeadingErrorFromGoalDegrees(Pose pose){
        if(pose.getX() > 72){
            return getHeadingErrorRed(pose);
        }
        else if(pose.getX() < 72){
            return getHeadingErrorBlue(pose);
        }
        return 0;

    }

    public double getHeadingErrorRed(Pose pose){
        return Math.toDegrees(pose.getHeading()) - Math.toDegrees(redGoalCenter.getHeading());
    }

    public double getHeadingErrorBlue(Pose pose){
        return Math.toDegrees(pose.getHeading()) - Math.toDegrees(blueGoalCenter.getHeading());
    }

     public void drawToPanels(Follower follower) {
        try {
            PanelsField field = PanelsField.INSTANCE;
            Drawing.drawRobot(follower.getPose());
            Drawing.drawPoseHistory(follower.getPoseHistory());
            Drawing.sendPacket();
            field.getField().update();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }
}
