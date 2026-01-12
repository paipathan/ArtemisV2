package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import dev.nextftc.core.commands.Command;

public class ShootArtifact extends Command {
    private final Robot robot;
    private final int shots;
    private final boolean staggerShots;
    private final ElapsedTime timer;

    private enum State {
        WAIT_FOR_SPINUP,
        SHOOTING,
        DONE
    }

    private State currentState;
    private int shotsFired;
    private boolean shotDetected;

    public ShootArtifact(Robot robot, int shots, boolean staggerShots) {
        this.robot = robot;
        this.shots = shots;
        this.staggerShots = staggerShots;
        this.timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        setInterruptible(true);
    }

    @Override
    public void start() {
        robot.intake.openGate().schedule();
        currentState = State.WAIT_FOR_SPINUP;
        shotsFired = 0;
        shotDetected = false;
        robot.outtake.setPredictedVelo(robot.getDistanceFromGoal()).schedule();

        Outtake.isBusy = true;
    }

    @Override
    public void update() {
        switch (currentState) {
            case WAIT_FOR_SPINUP:
                if (robot.outtake.getVelocity() >= robot.outtake.getPredictedVelo(robot.getDistanceFromGoal())) {
                    robot.intake.feed().schedule();
                    timer.reset();
                    shotDetected = false;
                    currentState = State.SHOOTING;
                }
                break;

            case SHOOTING:
                if (robot.outtake.getVelocity() < robot.outtake.getPredictedVelo(robot.getDistanceFromGoal()) - 100) {
                    shotDetected = true;
                }

                if (timer.time() > 2) {
                    shotDetected = true;
                }

                if (shotDetected) {
                    shotsFired++;

                    if (shotsFired >= shots) {
                        robot.intake.stop().schedule();
                        robot.outtake.stop().schedule();
                        currentState = State.DONE;
                    } else if (staggerShots) {
                        robot.intake.stop().schedule();
                        currentState = State.WAIT_FOR_SPINUP;
                    } else {
                        timer.reset();
                        shotDetected = false;
                    }
                }
                break;

            case DONE:
                break;
        }
    }

    @Override
    public boolean isDone() {
        return currentState == State.DONE;
    }

    @Override
    public void stop(boolean interrupted) {
        robot.intake.stop().schedule();
        robot.outtake.stop().schedule();
        Outtake.isBusy = false;

    }
}