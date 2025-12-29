package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="motor Test", group="TeleOp")
public class MotorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {



        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");
        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, "motor1");





        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            motor.setPower(-1);
            motor1.setPower(-1);
        }
    }
}
