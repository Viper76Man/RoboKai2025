package org.firstinspires.ftc.teamcode.Jack.Drive;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Jack.Motors.ArcShooterV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.IntakeV1;
import org.firstinspires.ftc.teamcode.Jack.Odometry.Constants;
import org.firstinspires.ftc.teamcode.Jack.Odometry.DecodeFieldLocalizer;

public class RobotV2 {
    //HARDWARE--------------------------------------------------------------------------------------
    public ArcShooterV1 arcShooter = new ArcShooterV1();
    public IntakeV1 intake = new IntakeV1();
    public Follower follower;
    public GamepadV1 gamepadV1 = new GamepadV1();
    public MecanumDriveOnly drive = new MecanumDriveOnly();
    //VARIABLES-------------------------------------------------------------------------------------
    public enum Mode {
        TELEOP,
        AUTONOMOUS
    }
    public boolean usePinpointForTeleOp = true;
    public boolean shooterOn = true;
    //OTHER-----------------------------------------------------------------------------------------
    public DecodeFieldLocalizer localizer = new DecodeFieldLocalizer();
    //----------------------------------------------------------------------------------------------
    public void init(Mode mode, HardwareMap hardwareMap, GamepadV1 gamepadV1){
        //INITIALIZATION----------------------------------------------------------------------------
        initHardware(hardwareMap, gamepadV1);
        intake.setDirection(RobotConstantsV1.intakeDirection);
        follower = Constants.createFollower(hardwareMap);
        //INIT-STUFF--------------------------------------------------------------------------------
        switch (mode){
            case AUTONOMOUS:
            case TELEOP:
                break;
        }
    }

    public void initHardware(HardwareMap hardwareMap, GamepadV1 gamepadV1){
        arcShooter.init(hardwareMap);
        intake.init(hardwareMap);
        drive.init(hardwareMap, gamepadV1);
        this.gamepadV1 = gamepadV1;
    }

    //INTAKE----------------------------------------------------------------------------------------
    public void intakeRun(){
        if(activateIntake()) {
            intake.setPower(RobotConstantsV1.INTAKE_POWER);
        }
    }

    public void intakeSwitchDirection(){
        intake.switchDirection();
    }
    //SHOOTER---------------------------------------------------------------------------------------
    public void shooterSwitchDirection(){
        arcShooter.switchDirection();
    }
    public void setShooterTargetRPM(int rpm){
        arcShooter.setTargetRPM(rpm);
    }
    public void shooterRun(){
        if(activateShooter()) {
            arcShooter.run();
        }
    }
    //FUNCTIONS-------------------------------------------------------------------------------------
    public double getDistanceFromBackLaunchZone(){
        return Math.abs(localizer.getDistanceFromLaunchZone(follower.getPose()));
    }
    public boolean activateIntake(){
        return !activateShooter();
    }

    public boolean activateShooter(){
        if(usePinpointForTeleOp) {
            return getDistanceFromBackLaunchZone() < RobotConstantsV1.maxLaunchZoneArcShooterDistance;
        }
        else {
            return shooterOn;
        }
    }

    public void toggleShooterNoPinpoint(){
        shooterOn = !shooterOn;
    }

    public void shootBall(){
    }


    public boolean inLaunchZone(){
        return localizer.isRobotInBackLaunchZone(follower.getPose());
    }
    //----------------------------------------------------------------------------------------------
    public void systemStatesUpdate(){
        shooterRun();
        intakeRun();
        gamepadV1.update();
        if(usePinpointForTeleOp){
            if(gamepadV1.isGamepadReady() && gamepadV1.options){
                toggleShooterNoPinpoint();
                gamepadV1.resetTimer();
            }
            else if(gamepadV1.buttonTimer.seconds() > 0.2 && gamepadV1.right_trigger >= 0.15){
                shootBall();
                gamepadV1.resetTimer();
            }
        }
    }
}
