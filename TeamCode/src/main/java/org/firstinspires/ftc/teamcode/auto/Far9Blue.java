package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Drawing;

import dev.nextftc.core.commands.CommandManager;

import dev.nextftc.core.commands.groups.SequentialGroup;

@Autonomous(name="[BLUE] Far 9", group="Auto")
public class Far9Blue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, Alliance.BLUE, gamepad1);
        Paths paths = new Paths(robot.follower);
        robot.follower.setStartingPose(new Pose(55.76360808709176, 7.614307931570755, Math.toRadians(90)));


        SequentialGroup autoRoutine = new SequentialGroup(
                robot.followPath(paths.Path1, 1)
        );

        robot.follower.breakFollowing();
        CommandManager.INSTANCE.cancelAll();
        autoRoutine.schedule();

        waitForStart();

        robot.follower.breakFollowing();
        CommandManager.INSTANCE.cancelAll();
        autoRoutine.schedule();

        while(opModeIsActive()) {
            robot.follower.update();
            CommandManager.INSTANCE.run();


            Drawing.init();
            Drawing.drawRobot(robot.follower.getPose());
            Drawing.drawPoseHistory(robot.follower.getPoseHistory());
            Drawing.sendPacket();
        }
    }

    public static class Paths {

        public PathChain Path1;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 8.000), new Pose(72.000, 72.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();
        }
    }




}