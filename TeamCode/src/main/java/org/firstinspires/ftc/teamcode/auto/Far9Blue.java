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

import java.nio.file.Paths;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;

@Autonomous(name="[BLUE] Far 9", group="Auto")
public class Far9Blue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.endPose = null;
        Turret.lastKnownPosition = null;
        Robot robot = new Robot(hardwareMap, Alliance.BLUE, gamepad1);

        Paths paths = new Paths(robot.follower);
        robot.follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));


        SequentialGroup poop = new SequentialGroup(
            robot.outtake.setPredictedVelo(robot.getDistanceFromGoal()),
            new WaitUntil(() -> {return Math.abs(robot.outtake.getVelocity() - robot.outtake.getPredictedVelo(robot.getDistanceFromGoal())) < 30;}),
            robot.intake.feed(),
            new Delay(2.5),
            robot.intake.stop(),
            robot.outtake.stop()
        );



        SequentialGroup autoRoutine = new SequentialGroup(
                    robot.followPath(paths.shootPreload, 1),
                    poop,
                    robot.autoIntake,
                    robot.followPath(paths.intake1, 0.95),
                    new Delay(0.6),
                    robot.followPath(paths.shoot1, 1),
                    robot.intake.stop(),
                    poop,
                    robot.autoIntake,
                    robot.followPath(paths.alignIntake2, 1),
                    robot.followPath(paths.intake2, 0.95),
                    new Delay(0.6),
                    robot.followPath(paths.shoot2, 1),
                    robot.intake.stop(),
                    poop,
                    robot.autoIntake,
                    robot.followPath(paths.alignIntake3, 1),
                    robot.followPath(paths.intake3, 0.95),
                    new Delay(0.6),
                    robot.followPath(paths.shoot3, 1),
                    robot.intake.stop(),
                    poop
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

            telemetry.addData("current velo", robot.outtake.getVelocity());
            telemetry.addData("target velo", robot.outtake.getPredictedVelo(robot.getDistanceFromGoal()));

            telemetry.update();

            Drawing.init();
            Drawing.drawRobot(robot.follower.getPose());
            Drawing.drawPoseHistory(robot.follower.getPoseHistory());
            Drawing.sendPacket();
        }

        robot.saveState();
    }



    public static class Paths {
        public PathChain shootPreload;
        public PathChain intake1;
        public PathChain shoot1;
        public PathChain alignIntake2;
        public PathChain intake2;
        public PathChain shoot2;
        public PathChain alignIntake3;
        public PathChain intake3;
        public PathChain shoot3;

        public Paths(Follower follower) {
            shootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),

                                    new Pose(56.200, 83.600)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .build();

            intake1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.200, 83.600),

                                    new Pose(16.9, 84.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            shoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.200, 84.000),

                                    new Pose(56.200, 83.600)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            alignIntake2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.200, 83.600),

                                    new Pose(43.000, 60.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            intake2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(43.000, 60.000),

                                    new Pose(10.2, 60.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(8.700, 60.000),

                                    new Pose(72.000, 72.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            alignIntake3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(72.000, 72.000),

                                    new Pose(42.500, 36.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            intake3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.500, 36.000),

                                    new Pose(7.000, 36.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            shoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(7.000, 36.000),

                                    new Pose(72.000, 72.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();
        }
    }









}