package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import dev.nextftc.core.commands.Command;

public class ShootArtifact extends Command {
    private final Robot robot;
    private final int shots;
    private final ElapsedTime timer;

    private enum State {
        START_OUTTAKE,
        WAIT_FOR_SPINUP,
        WAIT_FOR_SHOOT,
        STOP,
        DONE
    }

    private State currentState;
    private int shotsFired;

    public ShootArtifact(Robot robot, int shots) {
        this.robot = robot;
        this.shots = shots;
        this.timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        setInterruptible(true);
    }

    @Override
    public void start() {
        currentState = State.START_OUTTAKE;
        robot.intake.openGate().schedule();
        Outtake.isBusy = true;
        shotsFired = 0;
    }

    @Override
    public void update() {
        switch (currentState) {
            case START_OUTTAKE:
                robot.outtake.setManual(1500).schedule();
                currentState = State.WAIT_FOR_SPINUP;
                break;

            case WAIT_FOR_SPINUP:
                if (robot.outtake.getVelocity() >= 1475) {
                    robot.intake.start().schedule();
                    timer.reset();
                    currentState = State.WAIT_FOR_SHOOT;
                }
                break;

            case WAIT_FOR_SHOOT:
                if (robot.outtake.getVelocity() < 1400 || timer.time() > 2) {
                    robot.intake.stop().schedule();
                    shotsFired++;

                    if (shotsFired >= shots) {
                        currentState = State.STOP;
                    } else {
                        currentState = State.WAIT_FOR_SPINUP;
                    }
                }
                break;

            case STOP:
                robot.outtake.stop().schedule();
                robot.intake.stop().schedule();
                currentState = State.DONE;
                break;

            case DONE:
                robot.outtake.stop().schedule();
                robot.intake.stop().schedule();
                Outtake.isBusy = false;
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