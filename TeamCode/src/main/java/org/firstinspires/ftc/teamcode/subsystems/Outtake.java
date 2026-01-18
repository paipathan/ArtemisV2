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

        outtakeMotor.setVelocityPIDFCoefficients(123, 0, 0, 12);


        configureLut();
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

    public InstantCommand setRevSpeed(double distance) {
        return new InstantCommand(() -> {
            outtakeMotor.setVelocity(getPredictedVelo(distance) * 0.7);
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
        if(distance < 36.84 || distance > 141.25) {
            return (6.87514 * distance +1350.24284) - 30;
        } else {
            return lut.get(distance) - 45;
        }
    }


    public void configureLut() {
        lut = new InterpLUT();
        lut.add(36.84, 1600);
        lut.add(48.35, 1650);
        lut.add(60.01, 1700);
        lut.add(62.75, 1800);
        lut.add(71.33, 1850);
        lut.add(79.23, 1900);
        lut.add(79.48, 1900);
        lut.add(92.63, 2050);
        lut.add(108.92, 2200);
        lut.add(134.54, 2250);
        lut.add(138.77, 2200);
        lut.add(141.25, 2350);
        lut.createLUT();
    }

}
