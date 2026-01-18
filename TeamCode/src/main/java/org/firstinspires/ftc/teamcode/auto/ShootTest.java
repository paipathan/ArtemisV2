package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Alliance;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;


@Autonomous
public class ShootTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Robot.endPose = null;
        Turret.lastKnownPosition = null;
        Robot robot = new Robot(hardwareMap, Alliance.BLUE, gamepad1);


        SequentialGroup shoot = new SequentialGroup(
                robot.shootArtifact(3),
                robot.autoIntake,
                new Delay(3),
                robot.intake.stop(),
                robot.shootArtifact(3)
        );

        CommandManager.INSTANCE.cancelAll();
        shoot.schedule();
        waitForStart();

        while(opModeIsActive()) {
            CommandManager.INSTANCE.run();
        }
    }
}
