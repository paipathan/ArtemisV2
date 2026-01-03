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

    public int target = 2000;


    public Pose resetPose;
    public Pose goalPose;

    public double desiredTurretAngle = 0;

    public Robot(HardwareMap hwMap, Alliance alliance, Gamepad gamepad) {
        this.gamepad = gamepad;
        this.alliance = alliance;

        resetPose = alliance == Alliance.BLUE ? new Pose(8, 8, Math.toRadians(90)) : new Pose(8, 8, Math.toRadians(90)).mirror();
        goalPose = new Pose(6, 144-6);

        outtake = new Outtake(hwMap);
        intake = new Intake(hwMap);
        turret = new Turret(hwMap);
        led = new Led(hwMap);
        led.initializeArtboards();


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
        if(!Outtake.isBusy) {
            outtake.outtakeMotor.setVelocity(8.62791 * getDistanceFromGoal() + 1242.92601);
        }

        CommandManager.INSTANCE.run();
        BindingManager.update();
    }

    private void updateTurret() {
        Pose robotPose = follower.getPose();
        double robotHeading = robotPose.getHeading();


        double cosH = Math.cos(robotHeading);
        double sinH = Math.sin(robotHeading);

        double turretX = robotPose.getX() + (turret.TURRET_OFFSET_X * cosH - turret.TURRET_OFFSET_Y * sinH);
        double turretY = robotPose.getY() + (turret.TURRET_OFFSET_X * sinH + turret.TURRET_OFFSET_Y * cosH);

        double dx = goalPose.getX() - turretX;
        double dy = goalPose.getY() - turretY;

        double fieldTargetAngle = Math.atan2(dy, dx);
        desiredTurretAngle = normalize(fieldTargetAngle - robotHeading);

        double targetAngle;
        if (LimeLight.getLatestResult() != null) {
            double tx = LimeLight.getTX();
            double txRad = Math.toRadians(tx);
            if (Math.abs(tx) < 5.0) {
                double currentAngleRad = ticksToRadians(turret.turretMotor.getCurrentPosition());
                targetAngle = currentAngleRad + txRad;
            } else {
                targetAngle = desiredTurretAngle + txRad;
            }
        } else {
            targetAngle = desiredTurretAngle;
        }

        double targetTicks = radiansToTicks(targetAngle);
        turret.update(targetTicks);
    }

    public double radiansToTicks(double rad) {
        return (rad / (2.0 * Math.PI)) * turret.TICKS_PER_REV;
    }

    private double ticksToRadians(double ticks) {
        return (ticks / turret.TICKS_PER_REV) * (2.0 * Math.PI);
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
                    if(Outtake.isBusy) { // feed
                        intake.openGate().schedule();
                    } else {             // intake
                        intake.closeGate().schedule();
                        led.setState(Led.State.BLINK_RED);
                    }
                })
                .whenBecomesFalse(() -> {
                    intake.stop().schedule();
                    led.setState(Led.State.SOLID_RED);
                });

        Button toggleOuttake = button(() -> gamepad.right_bumper)
                .whenBecomesTrue(()-> {
                    intake.openGate().schedule();
                    led.setState(Led.State.BLINK_GREEN);
                })
                .whenBecomesFalse(() -> {
                    outtake.stop().schedule();
                    led.setState(Led.State.SOLID_GREEN);

                    gamepad.stopRumble();
                })
                .whenTrue(() -> {
                    outtake.setManual(8.62791 * getDistanceFromGoal() +1242.92601).schedule();

                    if(outtake.getVelocity() >= (8.62791 * getDistanceFromGoal() +1242.92601) && !Intake.isBusy) {
                        intake.start().schedule();
                    } else if(outtake.getVelocity() < (8.62791 * getDistanceFromGoal() +1242.92601) && Intake.isBusy) {
                        intake.stop().schedule();
                    }
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