package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.LimeLight;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Drawing;

import java.net.HttpURLConnection;


@TeleOp(name="[Test] TeleOp", group="TeleOp")
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.endPose = null;
        Turret.lastKnownPosition = null;
        Robot robot = new Robot(hardwareMap, Alliance.BLUE, gamepad1);

        robot.follower.setStartingPose(robot.resetPose);

        robot.follower.update();

        waitForStart();

        while(opModeIsActive()) {
            robot.periodic();

            telemetry.addData("Outtake target: ", robot.target);
            telemetry.addData("Outtake rpm: ", robot.outtake.outtakeMotor.getVelocity());
            telemetry.addData("turret: ", robot.turret.turretMotor.getCurrentPosition());
            telemetry.addData("Servo Stopper Position: ", robot.intake.servo.getPosition());
            telemetry.addData("Distance from goal:", robot.getDistanceFromGoal());
            telemetry.addData("Predicted:", robot.outtake.getPredictedVelo(robot.getDistanceFromGoal()));

            telemetry.update();

            Drawing.init();
            Drawing.drawRobot(robot.follower.getPose());
            Drawing.drawPoseHistory(robot.follower.getPoseHistory());
            Drawing.sendPacket();
        }
    }
}
