package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Drawing;

import dev.nextftc.core.commands.CommandManager;

import dev.nextftc.core.commands.groups.SequentialGroup;

@Autonomous(name="[BLUE] Far 9", group="Auto")
public class Far9Blue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.endPose = null;
        Turret.lastKnownPosition = null;
        Robot robot = new Robot(hardwareMap, Alliance.BLUE, gamepad1);

        Paths paths = new Paths(robot.follower);
        robot.follower.setStartingPose(new Pose(55.76360808709176, 7.614307931570755, Math.toRadians(90)));


        SequentialGroup autoRoutine = new SequentialGroup(
                robot.followPath(paths.shoot1, 1),
                robot.shootArtifact(3, true),
                robot.autoIntake,
                robot.followPath(paths.intake1, 1),
                robot.intake.stop(),
                robot.followPath(paths.shoot2, 1),
                robot.shootArtifact(3, true)
        );

        robot.follower.breakFollowing();
        CommandManager.INSTANCE.cancelAll();
        autoRoutine.schedule();

        waitForStart();

        robot.follower.breakFollowing();
        CommandManager.INSTANCE.cancelAll();
        autoRoutine.schedule();

        while(opModeIsActive()) {
            robot.autoPeriodic();

            Drawing.init();
            Drawing.drawRobot(robot.follower.getPose());
            Drawing.drawPoseHistory(robot.follower.getPoseHistory());
            Drawing.sendPacket();
        }

        robot.saveState();
    }

    public static class Paths {
        public PathChain shoot1;
        public PathChain intake1;
        public PathChain shoot2;

        public Paths(Follower follower) {
            shoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.800, 7.600),

                                    new Pose(55.800, 83.600)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .build();

            intake1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.800, 83.600),

                                    new Pose(15.000, 83.600)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.000, 83.600),

                                    new Pose(55.800, 83.600)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();
        }
    }




}