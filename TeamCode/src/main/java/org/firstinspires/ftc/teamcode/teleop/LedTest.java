package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Led;

@TeleOp
public class LedTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Led led = new Led(hardwareMap);

        led.initializeArtboards();

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.a) {
                led.setState(Led.State.SOLID_RED);
            } else if(gamepad1.b) {
                led.setState(Led.State.SOLID_GREEN);
            } else if(gamepad1.x) {
                led.setState(Led.State.BLINK_RED);
            } else if(gamepad1.y) {
                led.setState(Led.State.BLINK_GREEN);
            }
        }
    }
}