package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;


import dev.nextftc.core.commands.Command;

public class FollowPath extends Command {

    private final PathChain path;
    private final double maxPower;
    private final Follower follower;

    public FollowPath(PathChain path, Follower follower, double maxPower) {
        setInterruptible(true);
        this.path = path;
        this.maxPower = maxPower;
        this.follower = follower;
    }

    @Override
    public void start() {
        follower.followPath(path, maxPower, false);
    }

    @Override
    public void update() {}

    @Override
    public boolean isDone() {
        return follower.atParametricEnd();
    }

    @Override
    public void stop(boolean interrupted) {
        if (interrupted) {
            follower.breakFollowing();
        }
    }


}