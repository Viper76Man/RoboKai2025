package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import dev.nextftc.core.commands.Command;

public class CustomCommand extends Command {
    public Command actualCommand;
    public int ranCount = 0;


    public CustomCommand(Command command){
        this.actualCommand = command;
    }

    public boolean ranAlready(){
        return ranCount >= 1;
    }

    public int getRunCount(){
        return ranCount;
    }

    @Override
    public void start(){
        runCommand();
    }

    @Override
    public boolean isDone() {
        return actualCommand.isDone();
    }

    public void run(){
        runCommand();
    }

    private void runCommand(){
        actualCommand.schedule();
        ranCount += 1;
    }


}
