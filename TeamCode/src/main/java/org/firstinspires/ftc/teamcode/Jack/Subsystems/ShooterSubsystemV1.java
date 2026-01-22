package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Jack.Drive.Robot;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.ArcShooterV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV1;
import org.firstinspires.ftc.teamcode.Jack.Other.BallManager;
import org.firstinspires.ftc.teamcode.Jack.Servos.FlickerServoV1;
import org.firstinspires.ftc.teamcode.Jack.Servos.FlickerServoV2;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class ShooterSubsystemV1 implements Subsystem {

    public Robot.Mode mode;
    public ArcShooterV1 arcShooterV1 = new ArcShooterV1();
    public FlickerServoV2 flicker = new FlickerServoV2();

    public boolean busy = false;
    public IntakeSubsystemV1 intake;
    public BallManager manager;
    public HardwareMap hardwareMap;


    @Override
    public void initialize() {
        setHardwareMap(ActiveOpMode.hardwareMap());
    }

    @Override
    public void periodic() {
        //Loop
    }

    private void setHardwareMap(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public void init(HardwareMap hardwareMap_, Robot.Mode gamemode, IntakeSubsystemV1 intakeSubsystem, BallManager manager){
        this.mode = gamemode;
        this.intake = intakeSubsystem;
        this.manager = manager;
        this.hardwareMap = hardwareMap_;
        switch (mode) {
            case TELEOP:
                arcShooterV1.init(hardwareMap_, RobotConstantsV1.arcPIDs);
                break;
            case AUTONOMOUS:
                arcShooterV1.init(hardwareMap_, RobotConstantsV1.arcPIDsAuto);
                break;
        }
        flicker.init(hardwareMap_, RobotConstantsV1.flickerServoName);
    }

    public void shootTriple() {
        ShootBallTriple shoot = new ShootBallTriple();
        if(!busy) {
            shoot.schedule();
            busy = true;
        }
    }

    //SHOOTER---------------------------------------------------------------------------------------
    public class ShootBallTriple extends Command {
        public boolean flickerCycled = false;
        public boolean firedAlready = false;

        public ElapsedTime shootBallTimer = new ElapsedTime();

        public ShootBallTriple(){
            named("Shoot Triple");
        }


        @Override
        public void start() {
            shootBallTimer.reset();
            manager.setCurrentBall(1);
            busy = true;
        }

        @Override
        public void update() {
            arcShooterV1.run();
            intake.spindexer.run();
            flicker.update(intake.spindexer.isSpindexerReady());
            if(arcShooterV1.isInRange(30)) {
                if (flickerCycled) {
                    flickerCycled = false;
                    manager.next();
                } else {
                    if (isFlickerDown() && !firedAlready && intake.spindexer.isSpindexerReady()) {
                        setFlickerUp();
                        firedAlready = true;
                    }
                    if (isFlickerDown() && firedAlready) {
                        manager.setEmpty(manager.getCurrentBall());
                        flickerCycled = true;
                    }
                }
            }
        }


        @Override
        public boolean isDone() {
            return (manager.isEmpty(1) && manager.isEmpty(2) && manager.isEmpty(3)) || ActiveOpMode.isStopRequested();
        }

        @Override
        public void stop(boolean interrupted){
            arcShooterV1.setTargetRPM(RobotConstantsV1.SHOOTER_IDLE_RPM);
            busy = false;
        }

        public boolean isFlickerMoving(){
            return flicker.getState() != FlickerServoV2.State.IDLE;
        }


        public boolean isFlickerTravelingDown(){
            return flicker.getState() == FlickerServoV2.State.TRAVEL_DOWN;
        }

        public boolean isFlickerDown(){
            return flicker.getState() == FlickerServoV2.State.IDLE;
        }

        public void setFlickerUp(){
            flicker.setState(FlickerServoV2.State.DOWN);
        }
    }

}
