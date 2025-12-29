package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="[TEST] Motor Direction", group="Test")
public class MotorDirectionTest extends OpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        // Match these to your Constants.java settings
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        // Test each motor individually
        if (gamepad1.y) {
            frontLeft.setPower(0.3);
            telemetry.addLine("Testing: FRONT LEFT");
        } else if (gamepad1.b) {
            frontRight.setPower(0.3);
            telemetry.addLine("Testing: FRONT RIGHT");
        } else if (gamepad1.x) {
            backLeft.setPower(0.3);
            telemetry.addLine("Testing: BACK LEFT");
        } else if (gamepad1.a) {
            backRight.setPower(0.3);
            telemetry.addLine("Testing: BACK RIGHT");
        } else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            telemetry.addLine("Y=FL  B=FR  X=BL  A=BR");
        }

        telemetry.update();
    }
}