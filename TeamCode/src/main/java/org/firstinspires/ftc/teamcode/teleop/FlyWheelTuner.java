package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import dev.nextftc.core.commands.CommandManager;


@TeleOp
public class FlyWheelTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        double p = 123;
        double i = 0;
        double d = 0;
        double f = 12;


        double target = 2000;


        DcMotorEx outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");

        Intake intake = new Intake(hardwareMap);

        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        outtakeMotor.setVelocityPIDFCoefficients(p, i, d, f);

        waitForStart();


        while(opModeIsActive()) {


            if(gamepad1.leftBumperWasPressed()) {
                intake.start().schedule();
            }


            if(gamepad1.leftBumperWasReleased()) {
                intake.stop().schedule();
            }

            if(gamepad1.bWasPressed()) {
                target = target + 50;
            }

            if(gamepad1.xWasPressed()) {
                target = target - 50;
            }


            if(gamepad1.dpadUpWasPressed()) {
                p = p + 0.1;
            }

            if(gamepad1.dpadDownWasPressed()) {
                p = p - 0.1;
            }

            if(gamepad1.yWasPressed()) {
                f = f + 0.1;
            }

            if(gamepad1.aWasPressed()) {
                f = f - 0.1;
            }

            outtakeMotor.setVelocityPIDFCoefficients(p, i, d, f);
            outtakeMotor.setVelocity(target);


            telemetry.addData("Target: ", target);
            telemetry.addData("Current: ", outtakeMotor.getVelocity());
            telemetry.addData("P: ", p);
            telemetry.addData("F: ", f);
            telemetry.update();

            CommandManager.INSTANCE.run();

        }
    }
}
