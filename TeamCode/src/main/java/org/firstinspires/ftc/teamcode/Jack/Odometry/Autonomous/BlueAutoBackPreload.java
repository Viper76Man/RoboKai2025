package org.firstinspires.ftc.teamcode.Jack.Odometry.Autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.GamepadV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.Robot;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotV3;
import org.firstinspires.ftc.teamcode.Jack.Odometry.BlueAutoPathsV2;
import org.firstinspires.ftc.teamcode.Jack.Odometry.CustomFollower;
import org.firstinspires.ftc.teamcode.Jack.Odometry.DecodeFieldLocalizer;
import org.firstinspires.ftc.teamcode.Jack.Other.DecodeAprilTag;
import org.firstinspires.ftc.teamcode.Jack.Other.Drawing;
import org.firstinspires.ftc.teamcode.Jack.Other.Range;

@Autonomous
public class BlueAutoBackPreload extends LinearOpMode {
    public CustomFollower follower;
    public BlueAutoPathsV2 pathsV2 = new BlueAutoPathsV2();
    public DecodeAprilTag obeliskTag;
    public RobotV3 robot = new RobotV3();
    //FAKE STUFF------------------------------------------------------------------------------------
    public int ballsFired = 0;
    public ElapsedTime ballTimer = new ElapsedTime();

    public enum PathStates {
        START,
        TO_SHOOT,
        SHOOT_SET_1,
        TO_PICKUP_1,
        TURN_TO_PICKUP_1,
        PICKUP_1,
        BACK_TO_SHOOT_1,
        SHOOT_SET_2
    }

    public enum ActionStates {
        DRIVE_TO_SHOOT,
        SHOOT_1,
        DRIVE_TO_BALLS_1
    }

    public PathStates pathState;
    public ActionStates actionState;
    public boolean actionIsSet = false;
    public GamepadV1 gamepadV1 = new GamepadV1();

    @Override
    public void runOpMode() {
        initHardware();
        pathState = PathStates.START;
        actionState = ActionStates.DRIVE_TO_SHOOT;
        follower.setStartingPose(BlueAutoPathsV2.startPoseFar);
        if(obeliskTag != null) {
            telemetry.addData("Latest tag: ", obeliskTag.name());
        }
        waitForStart();
        while (opModeIsActive()){
            log();
            autoPathUpdate();
            systemStatesUpdate();
            if(RobotConstantsV1.panelsEnabled){
                draw();
            }
        }
    }

    public void initHardware(){
        follower = new CustomFollower(hardwareMap);
        pathsV2.buildPaths();
        gamepadV1.init(gamepad1, 0.3);
        robot.init(hardwareMap, gamepadV1,  Robot.Mode.AUTONOMOUS, Robot.Alliance.BLUE);
        robot.setSystemState(RobotV3.State.INTAKE_BALL_1);
    }

    public void autoPathUpdate(){
        follower.update(telemetry);
        telemetry.addData("Pose: ", follower.follower.getPose());
        switch (pathState) {
            case START:
                setPathState(PathStates.TO_SHOOT);
                if (!actionIsSet){
                    robot.setSystemState(RobotV3.State.INTAKE_BALL_1);
                    actionIsSet = true;
                 }
                break;
            case TO_SHOOT:
                if(!follower.isBusy()){
                    follower.setCurrentPath(BlueAutoPathsV2.outOfStartFar);
                    setPathState(PathStates.SHOOT_SET_1);
                    break;
                }
                break;
            case SHOOT_SET_1:
                if(follower.follower.getCurrentTValue() >= 1 && actionState != ActionStates.DRIVE_TO_BALLS_1) {
                    if(ballTimer.seconds() > 1.5) {
                        robot.setSystemState(RobotV3.State.SHOOT_BALL_1);
                        setActionState(ActionStates.SHOOT_1);
                    }
                }
                else {
                    if(follower.follower.getCurrentTValue() < 1){
                        ballTimer.reset();
                    }
                }
                if(actionState == ActionStates.DRIVE_TO_BALLS_1){
                    setPathState(PathStates.TO_PICKUP_1);
                    ballsFired = 0;
                }
                break;
        }
    }

    public void systemStatesUpdate(){
        robot.systemStatesUpdate();
        switch (actionState){
            case DRIVE_TO_SHOOT:
                break;
            case SHOOT_1:
                if(ballsFired < 3 && robot.isSpindexerReady() && !robot.fire && new Range((RobotConstantsV1.SHOOTER_TARGET_RPM), 10).isInRange(robot.arcShooter.getVelocityRPM()) && robot.currentBall != ballsFired + 1){
                    fireBall();
                }
                if(ballsFired >= 3){
                    setActionState(ActionStates.DRIVE_TO_BALLS_1);
                }
                break;
        }
    }

    public void draw(){
        Drawing.drawRobot(follower.follower.getPose());
        Drawing.drawPoseHistory(follower.follower.getPoseHistory());
        Drawing.sendPacket();
    }

    public void fireBall(){
        ballsFired += 1;
        robot.fire = true;
    }

    public void setPathState(PathStates pathState){
        this.pathState = pathState;
    }
    public void setActionState(ActionStates actionState){
        this.actionState = actionState;
        actionIsSet = false;
    }

    public void log(){
        telemetry.addLine("State: " + pathState.name());
        telemetry.addLine("Action: " + actionState.name());
        telemetry.addLine("Balls fired: " + ballsFired);
        telemetry.addLine("T-value: " + follower.follower.getCurrentTValue());
        telemetry.addLine("Busy? " + follower.isBusy());
    }

}
