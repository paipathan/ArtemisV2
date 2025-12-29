package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.hardware.impl.ServoEx;

public class Intake {

    public DcMotorEx backMotor;
    public DcMotorEx frontMotor;

    public Servo servo;

    public double GATE_CLOSE = 1;
    public double GATE_OPEN = 0;

    public Intake(HardwareMap hwMap) {
        backMotor = hwMap.get(DcMotorEx.class, "backIntakeMotor");
        backMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontMotor = hwMap.get(DcMotorEx.class, "frontIntakeMotor");
        frontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        servo = hwMap.get(Servo.class, "servoStopper");
        servo.setDirection(Servo.Direction.FORWARD);
    }


    public InstantCommand stop() {
        return new InstantCommand(() -> {
            backMotor.setPower(0);
            frontMotor.setPower(0);
        });
    }

    public InstantCommand start() {
        return new InstantCommand(()-> {
            backMotor.setPower(1);
            frontMotor.setPower(1);
        });
    }

    public InstantCommand startIntake() {
        return new InstantCommand(()-> {
            backMotor.setPower(1);
            frontMotor.setPower(1);

            servo.setPosition(GATE_CLOSE);
        });
    }

    public InstantCommand feed() {
        return new InstantCommand(()-> {
            backMotor.setPower(0);
            frontMotor.setPower(0);

            servo.setPosition(GATE_OPEN);
        });
    }

    public InstantCommand reverse() {
        return new InstantCommand(()-> {
            backMotor.setPower(-1);
            frontMotor.setPower(-1);
        });
    }

    public InstantCommand openGate() {
        return new InstantCommand(()-> {
            servo.setPosition(GATE_OPEN);
        });
    }

    public InstantCommand closeGate() {
        return new InstantCommand(()-> {
            servo.setPosition(GATE_CLOSE);
        });
    }
}

// open pos: 0
// close pos: 1
