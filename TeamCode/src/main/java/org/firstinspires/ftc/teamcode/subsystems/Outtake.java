package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import dev.nextftc.core.commands.utility.InstantCommand;

public class Outtake {

    public DcMotorEx outtakeMotor;

    public InterpLUT lut;

    public static boolean isBusy = false;


    public Outtake(HardwareMap hwMap) {
        outtakeMotor = hwMap.get(DcMotorEx.class, "outtakeMotor");

        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        configureLUT();
    }


    public double getVelocity () {
        return outtakeMotor.getVelocity();
    }


    public InstantCommand setManual(double velocity) {
        return new InstantCommand(() -> {
            outtakeMotor.setVelocity(velocity);
            isBusy = true;
        });
    }

    public InstantCommand setPredictedVelo(double distance) {
        return new InstantCommand(() -> {
            outtakeMotor.setVelocity(getPredictedVelo(distance));
            isBusy = true;
        });
    }

    public InstantCommand stop() {
        return new InstantCommand(() -> {
            outtakeMotor.setVelocity(0);
            isBusy = false;
        });
    }

    public InstantCommand reverse(int velocity) {
        return new InstantCommand(() -> {
            outtakeMotor.setVelocity(-velocity);
            isBusy = true;
        });
    }

    public double getPredictedVelo(double distance) {
        return lut.get(distance);
    }


    public void configureLUT() {
        lut = new InterpLUT();
        lut.add(53, 1700);
        lut.add(70, 1800);
        lut.add(90, 2050);
        lut.add(115, 2300);
        lut.add(147, 2400);
        lut.add(150, 2600);
        lut.createLUT();
    }
}
