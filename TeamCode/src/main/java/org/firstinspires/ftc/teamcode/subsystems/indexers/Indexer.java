package org.firstinspires.ftc.teamcode.subsystems.indexers;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.ForwardSubsystem;

public abstract class Indexer extends ForwardSubsystem {

    public enum State{
        HOLD,
        FEED,
        UNJAM
    }

    protected State currentState = State.HOLD;
    protected State desiredState = State.HOLD;

    protected abstract void onHold();
    protected abstract void onFeed();
    protected abstract void onUnjam();


    @Override
    public void act() {
        if (desiredState != currentState) {
            switch (desiredState) {
                case HOLD:
                    onHold();
                    currentState = State.HOLD;
                    break;
                case FEED:
                    onFeed();
                    currentState = State.FEED;
                    break;
                case UNJAM:
                    onUnjam();
                    currentState = State.UNJAM;
                    break;
            }
        }
    }

    public Command hold(){
        return new InstantCommand(() -> desiredState = State.HOLD);
    }
    public Command feed(){
        return new InstantCommand(() -> desiredState = State.FEED);
    }
    public Command unjam(){
        return new InstantCommand(() -> desiredState = State.UNJAM);
    }


}
