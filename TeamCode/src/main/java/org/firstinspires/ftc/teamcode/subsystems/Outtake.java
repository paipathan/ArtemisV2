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

//        configureLUT();
    }



    public InstantCommand setManual(int velocity) {
        return new InstantCommand(() -> {
            outtakeMotor.setVelocity(velocity);
            isBusy = true;
        });
    }

    public InstantCommand setPredictedVelo(double distance) {
        return new InstantCommand(() -> {
            outtakeMotor.setVelocity(lut.get(distance));
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


//    public void configureLUT() {
//        lut = new InterpLUT();
//        lut.add(0, 0);
//        lut.add(0, 0);
//        lut.add(0, 0);
//        lut.add(0, 0);
//        lut.add(0, 0);
//
//        lut.createLUT();
//    }
}
