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
        if(distance < 30.61 || distance > 146.61) {
            return 7.40945 * distance + 1336.69712;
        } else {
            return lut.get(distance);
        }
    }


    public void configureLut() {
        lut = new InterpLUT();
        lut.add(30.61, 1620);
        lut.add(47.95, 1700);
        lut.add(53.81, 1730);
        lut.add(65.89, 1850);
        lut.add(79.52, 1900);
        lut.add(104.34, 2000);
        lut.add(114.57, 2050);
        lut.add(125.94, 2300);
        lut.add(135.42, 2450);
        lut.add(146.61, 2470);
        lut.createLUT();
    }

}
