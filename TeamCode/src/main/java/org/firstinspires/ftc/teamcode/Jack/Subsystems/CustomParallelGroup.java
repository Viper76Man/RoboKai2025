package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import java.util.Arrays;
import java.util.List;

import dev.nextftc.core.commands.groups.ParallelGroup;

public class CustomParallelGroup {
    public ParallelGroup group;
    public List<CustomCommand> commandsList;
    public int runCount = 0;
    private int index = 1;
    public CustomParallelGroup(CustomCommand... commands){
        group = new ParallelGroup(commands[0]);
        while (index < commands.length){
            group.and(commands[index]);
            index += 1;
        }
    }

    public void schedule(){
        runCount += 1;
        group.schedule();
    }

    public boolean isDone(){
        return group.isDone();
    }

    public boolean hasRun(){
        return runCount > 0;
    }



}
