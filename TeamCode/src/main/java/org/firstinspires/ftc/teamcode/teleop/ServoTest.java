package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo stopper= hardwareMap.get(Servo.class, "servoStopper");
        double currentPos = 0;


        waitForStart();

        while(opModeIsActive()) {

            telemetry.addData("Stopper position: ", stopper.getPosition());
            telemetry.update();

            if(gamepad1.left_bumper) {
                stopper.setPosition(0.5);
            }

            if(gamepad1.right_bumper) {
                stopper.setPosition(0);
            }



        }
    }
}
