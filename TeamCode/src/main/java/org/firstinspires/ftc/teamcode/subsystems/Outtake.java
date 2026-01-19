package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

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
        if(distance < 28.19 || distance > 169.9) {
            return -0.000152404 * Math.pow(distance,3) + 0.0735528 * Math.pow(distance, 2) -1.65181* distance +1556.83594;
        } else {
            return lut.get(distance) + 20;
        }
    }

    public double tuningRPM(int velocity) {
        if (gamepad1.dpadUpWasPressed()) {
            return velocity + 100;
        } else if (gamepad1.dpadDownWasPressed()) {
            return velocity - 100;
        } else {
            return velocity;
        }
    }


    public void configureLut() {
        lut = new InterpLUT();
        lut.add(28.19, 1550);
        lut.add(40.491, 1600);
        lut.add(46.45, 1650);
        lut.add(54.57, 1675);
        lut.add(80.33, 1775);
        lut.add(83.89, 1850);
        lut.add(125.3, 2250);
        lut.add(135.98, 2275);
        lut.add(149.37, 2450);
        lut.add(169.9, 2650);
        lut.createLUT();
    }

}
