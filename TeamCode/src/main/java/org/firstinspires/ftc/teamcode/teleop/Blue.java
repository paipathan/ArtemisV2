package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Led;
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


            telemetry.addData("Outtake target: ", robot.target);
            telemetry.addData("Outtake rpm: ", robot.outtake.outtakeMotor.getVelocity());
            telemetry.addData("Distance from goal:", robot.getDistanceFromGoal());

            telemetry.update();

            if(gamepad1.dpad_up) {
                robot.target = robot.target + 50;
            } else if (gamepad1.dpad_down) {
                robot.target = robot.target - 50;
            }


            Drawing.init();

            Drawing.drawRobot(robot.follower.getPose());
            Drawing.drawPoseHistory(robot.follower.getPoseHistory());
            Drawing.sendPacket();
        }
    }
}
