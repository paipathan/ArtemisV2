package org.firstinspires.ftc.teamcode;

import static dev.nextftc.bindings.Bindings.button;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Led;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Alliance;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;

public class Robot {

    public Follower follower;
    public Gamepad gamepad;
    public Alliance alliance;

    public Outtake outtake;
    public Intake intake;
    public Turret turret;
    public Led led;

    public Pose resetPose;
    public Pose goalPose;

    public double desiredTurretAngle = 0;

    public Robot(HardwareMap hwMap, Alliance alliance, Gamepad gamepad) {
        this.gamepad = gamepad;
        this.alliance = alliance;

        resetPose = alliance == Alliance.BLUE ? new Pose(8, 8, Math.toRadians(90)) : new Pose(8, 8, Math.toRadians(90)).mirror();
        goalPose = new Pose(12, 137);

        outtake = new Outtake(hwMap);
        intake = new Intake(hwMap);
        turret = new Turret(hwMap);
//        led = new Led(hwMap);


        follower = Constants.createFollower(hwMap);
        follower.startTeleopDrive();
        follower.setStartingPose(resetPose);
        follower.update();

        configureKeybinds();
    }

    public void periodic() {
        follower.setTeleOpDrive(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x, true);
        follower.update();


        updateTurret();
//        if(!Outtake.isBusy) {
//            outtake.outtakeMotor.setVelocity(1000);
//        }

        CommandManager.INSTANCE.run();
        BindingManager.update();
    }

    private void updateTurret() {
        Pose robotPose = follower.getPose();

        double robotHeading = robotPose.getHeading();

        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();

        double fieldTargetAngle = Math.atan2(dy, dx);

        desiredTurretAngle = normalize(fieldTargetAngle - robotHeading);

        double targetAngle;

        if (LimeLight.getLatestResult() != null) {
            double txRad = Math.toRadians(LimeLight.getTX());
            targetAngle = desiredTurretAngle - txRad;
        } else {
            targetAngle = desiredTurretAngle;
        }

        double targetTicks = radiansToTicks(targetAngle);
        turret.update(targetTicks);
    }

    public double radiansToTicks(double rad) {
        return (rad / (2.0 * Math.PI)) * turret.TICKS_PER_REV;
    }

    public static double normalize(double angleRadians) {
        double angle = angleRadians % (Math.PI * 2D);
        if (angle <= -Math.PI) angle += Math.PI * 2D;
        if (angle > Math.PI) angle -= Math.PI * 2D;
        return angle;
    }

    public void configureKeybinds() {
        Button toggleIntake = button(() -> gamepad.left_bumper)
                .whenBecomesTrue(() -> {
                    intake.start().schedule();
                    intake.closeGate().schedule();
                })
                .whenBecomesFalse(() -> {
                    intake.stop().schedule();
                });

        Button toggleOuttake = button(() -> gamepad.right_bumper)
                .whenBecomesTrue(()-> {
                    outtake.setManual(2200).schedule();
                    intake.openGate().schedule();
                })
                .whenBecomesFalse(() -> {
                    outtake.stop().schedule();
                });

        Button resetPoseButton = button(() -> gamepad.a)
                .whenBecomesTrue(() -> {
                    follower.setPose(resetPose);
                    follower.update();
                });
    }

    public Command followPath(PathChain path, double maxPower) {
        return new FollowPath(path, this.follower, maxPower);
    }

    public double getDistanceFromGoal() {
        return follower.getPose().distanceFrom(goalPose);
    }
}