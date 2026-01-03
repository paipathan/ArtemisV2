package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.pedropathing.control.PIDFController;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {

    public DcMotorEx turretMotor;

    private PIDFController coarsePIDF;
    private PIDFController finePID;

    public int MIN_TICKS = -689;
    public int MAX_TICKS =  689;
    public int TICKS_PER_REV = 847*2;



    public double TURRET_OFFSET_X = 0;
    public double TURRET_OFFSET_Y   = -6.457;

    private static final double SWITCH_THRESHOLD = 25;

    public Turret(HardwareMap hwMap) {
        turretMotor = hwMap.get(DcMotorEx.class, "turretMotor");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        coarsePIDF = new PIDFController(new PIDFCoefficients(0.03, 0,0,0));
        finePID = new PIDFController( new PIDFCoefficients(0.04,0,0,0));
    }

    public void update(double targetTicks) {
        targetTicks = clamp(targetTicks, MIN_TICKS, MAX_TICKS);

        double error = targetTicks - turretMotor.getCurrentPosition();

        double power;

        if (Math.abs(error) > SWITCH_THRESHOLD) {
            coarsePIDF.updateError(error);
            coarsePIDF.updateFeedForwardInput(Math.signum(error));
            power = coarsePIDF.run();
        } else {
            finePID.updateError(error);
            power = finePID.run();
        }

        turretMotor.setPower(clamp(power, -1.0, 1.0));
    }
}
