package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Drawing;

import kotlin.time.Instant;


@TeleOp(name="[BLUE] TeleOp", group="TeleOp")
public class Blue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, Alliance.BLUE, gamepad1);
        waitForStart();

        while(opModeIsActive()) {
            robot.periodic();

            telemetry.addData("Turret current ticks: ", robot.turret.turretMotor.getCurrentPosition());
            telemetry.update();

            telemetry.addData("dx", robot.goalPose.getX() - robot.follower.getPose().getX());
            telemetry.addData("dy", robot.goalPose.getY() - robot.follower.getPose().getY());
            telemetry.addData("Field target angle (deg)", Math.toDegrees(Math.atan2(
                    robot.goalPose.getY() - robot.follower.getPose().getY(),
                    robot.goalPose.getX() - robot.follower.getPose().getX()
            )));
            telemetry.addData("Desired turret (deg)", Math.toDegrees(robot.desiredTurretAngle));
            telemetry.addData("Target ticks (before clamp)", robot.radiansToTicks(robot.desiredTurretAngle));


            Drawing.init();
            Drawing.drawRobot(robot.follower.getPose());
            Drawing.drawPoseHistory(robot.follower.getPoseHistory());
            Drawing.sendPacket();
        }
    }
}
