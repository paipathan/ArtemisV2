package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.LimeLight.distanceFromTag;
import static org.firstinspires.ftc.teamcode.LimeLight.getRobotPose;
import static dev.nextftc.bindings.Bindings.button;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.commands.ShootArtifact;
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
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;

public class Robot {

    public Follower follower;
    public Gamepad gamepad;
    public Alliance alliance;

    public Outtake outtake;
    public Intake intake;
    public Turret turret;
//    public Led led;

    private long lastValidLimelightTime = 0;
    private static final long LIMELIGHT_TIMEOUT_MS = 750;

    public double target = 2000;

    public Pose resetPose;
    public Pose goalPose;

    public static Pose endPose;

    public double desiredTurretAngle = 0;

    public InstantCommand autoIntake;

    public Pose bluestartpose = new Pose(8.299065, 8.299065, Math.toRadians(90));

    public Robot(HardwareMap hwMap, Alliance alliance, Gamepad gamepad) {
//        LimeLight.init(hwMap);
        this.gamepad = gamepad;
        this.alliance = alliance;

        resetPose = alliance == Alliance.BLUE ? bluestartpose : bluestartpose.mirror();
        goalPose = alliance == Alliance.BLUE ? new Pose(11, 138) : new Pose(11, 138).mirror();

        outtake = new Outtake(hwMap);
        intake = new Intake(hwMap);
        turret = new Turret(hwMap);
//        led = new Led(hwMap);
//        led.initializeArtboards();

        follower = Constants.createFollower(hwMap);
        follower.startTeleopDrive();
        follower.update();

        configureKeybinds();


        autoIntake = new InstantCommand(() -> {
            intake.closeGate().schedule();
            intake.start().schedule();
        });

//        if(endPose != null) {
//            follower.setPose(endPose);
//        } else {
//            follower.setPose(resetPose);
//        }

    }

    public void periodic() {
        follower.setTeleOpDrive(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x, true);
        follower.update();

        updateTurret();

        if(!Outtake.isBusy) outtake.outtakeMotor.setVelocity(outtake.getPredictedVelo(getDistanceFromGoal()) * 0.7);

        CommandManager.INSTANCE.run();
        BindingManager.update();
    }

    public void autoPeriodic() {
        follower.update();

        updateTurret();

        if(!Outtake.isBusy) {
            outtake.setRevSpeed(getDistanceFromGoal()).schedule();
        }

        CommandManager.INSTANCE.run();
        BindingManager.update();
    }

    public void updateTurret() {
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
//
//        if (LimeLight.validResult()) {
//            lastValidLimelightTime = System.currentTimeMillis();
//        }

        boolean hasRecentTarget = (System.currentTimeMillis() - lastValidLimelightTime) < LIMELIGHT_TIMEOUT_MS;

//        if (Outtake.isBusy && hasRecentTarget) {
//            if (LimeLight.validResult()) {
//                double tx = LimeLight.getTX();
//                double kp = 0.035;
//                double power = tx * kp;
//                turret.turretMotor.setPower(clamp(power, -1.0, 1.0));
//            } else {
//                turret.turretMotor.setPower(0);
//            }
//            return;
//        }
        double targetTicks = radiansToTicks(desiredTurretAngle);
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

        Button x = button(() -> gamepad.x)
                .whenBecomesTrue(() -> {
                    intake.servo.setPosition(intake.servo.getPosition() + 0.01);
                });

        Button b = button(() -> gamepad.b)
                .whenBecomesTrue(() -> {
                    intake.servo.setPosition(intake.servo.getPosition() - 0.01);
                });

       /* Button dpadup = button(() -> gamepad.dpad_up)
                .whenBecomesTrue(() -> {
                    target = target + 25;
                });

        Button dpaddown = button(() -> gamepad.dpad_down)
                .whenBecomesTrue(() -> {
                    target = target - 25;
                });*/

        Button toggleIntake = button(() -> gamepad.left_bumper)
                .whenBecomesTrue(() -> {
                    intake.start().schedule();
//                    led.setState(Led.State.BLINK_RED);
                })
                .whenBecomesFalse(() -> {
                    intake.stop().schedule();
                    // led.setState(Led.State.SOLID_RED);
                });

        Button toggleOuttake = button(() -> gamepad.right_bumper)
            .whenBecomesTrue(()-> {
                intake.openGate().schedule();
                outtake.setPredictedVelo(getDistanceFromGoal()).schedule();
            }).whenBecomesFalse(() -> {
                intake.stop().schedule();
                intake.closeGate().schedule();
                outtake.stop().schedule();
            })
                .whenTrue(() -> {
                    if(Math.abs(outtake.getVelocity() - outtake.getPredictedVelo(getDistanceFromGoal())) < 15 && !Intake.isBusy) {
                        intake.start().schedule();
                    } else if(Math.abs(outtake.getVelocity() - outtake.getPredictedVelo(getDistanceFromGoal())) > 15 && Intake.isBusy) {
                        intake.stop().schedule();
                    }
                });



//        Button toggleOuttake = button(() -> gamepad.right_bumper)
//                .whenBecomesTrue(()-> {
//                    intake.openGate().schedule();
//                    outtake.setPredictedVelo(getDistanceFromGoal()).schedule();
//                   // led.setState(Led.State.BLINK_GREEN);
//                })
//                .whenBecomesFalse(() -> {
//                    intake.closeGate().schedule();
//                    outtake.stop().schedule();
//                   // led.setState(Led.State.SOLID_GREEN);
//
//                    gamepad.stopRumble();
//                })
//                .whenTrue(() -> {
//                    if(Math.abs(outtake.getVelocity() - outtake.getPredictedVelo(getDistanceFromGoal())) < 50) {
//                        gamepad.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
//                    } else {
//                        gamepad.stopRumble();
//                    }
//                });

        Button resetPoseButton = button(() -> gamepad.a)
                .whenBecomesTrue(() -> {
                    follower.setPose(resetPose);
                    follower.update();
                });
    }

    public void saveState() {
        Robot.endPose = follower.getPose();
        Turret.lastKnownPosition = turret.getTurretPosition();
    }

    public Command shootArtifact(int shots) {
        return new ShootArtifact(this, shots);
    }

    public Command followPath(PathChain path, double maxPower) {
        return new FollowPath(path, this.follower, maxPower);
    }

    public double getDistanceFromGoal() {
        return follower.getPose().distanceFrom(goalPose);
    }

    public double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}