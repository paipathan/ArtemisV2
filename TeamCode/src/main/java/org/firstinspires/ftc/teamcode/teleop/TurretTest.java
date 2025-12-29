package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.LimeLight;

@TeleOp(name="Turret Test", group="TeleOp")
public class TurretTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDFController coarsePIDF = new PIDFController(new PIDFCoefficients(0.05, 0,0,0));
        PIDFController finePIDF = new PIDFController( new PIDFCoefficients(0.03,0,0,0));

        LimeLight.init(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

//            if (LimeLight.getLatestResult() != null) {
//                double tx = -LimeLight.getTX();
//                if(Math.abs(tx) > 15) {
//                    coarsePIDF.updateError(tx);
//                    coarsePIDF.updateFeedForwardInput(Math.signum(tx));
//                    motor.setPower(coarsePIDF.run());
//                } else {
//                    finePIDF.updateError(tx);
//                    motor.setPower(finePIDF.run());
//                } 1145,
//            } else {
//                motor.setPower(0);
//            }


            telemetry.addData("TX: ", LimeLight.getTX());
            telemetry.addData("Turret ticks: ", motor.getCurrentPosition());


            
            telemetry.update();
        }
    }
}
