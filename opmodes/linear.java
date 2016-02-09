package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by MthSci.Student on 1/25/2016.
 */
public class linear extends OpMode {

    /**
     * Constructor
     */
    public linear() {

    }

    //Motor object variables
    DcMotor motorRight, motorLeft, spinner, belts, lift, winch;

    //Servo object variables
    Servo linearLift, leftGate, rightGate, climberServoLeft, climberServoRight, churro, climberRotate, climberElbow;


    final static double LIFT_DELTA = .001;

    //Motor power variables
    double FORWARD_COLLECTOR_POWER = 1.0;
    double BACKWARD_COLLECTOR_POWER = -1.0;
    double FORWARD_SCORER_POWER = 1.0;
    double BACKWARD_SCORER_POWER = -1.0;
    double UP_LIFT_POWER = 1.0;
    double DOWN_LIFT_POWER = -1.0;

    //Servo position variables
    double LINEAR_LIFT_UP = .7;
    double LINEAR_LIFT_DOWN = .2;
    double LEFT_GATE_UP = .5;
    double LEFT_GATE_DOWN = 0.0;
    double RIGHT_GATE_UP = .1;
    double RIGHT_GATE_DOWN = .7;
    double LEFT_CLIMBER_UP = 0.0;
    double LEFT_CLIMBER_DOWN = .7;
    double RIGHT_CLIMBER_UP = .9;
    double RIGHT_CLIMBER_DOWN = 0.0;
    double CHURRO_UP = .8;
    double CHURRO_DOWN = .1;

    double liftPosition = 0.2;
    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {





        linearLift = hardwareMap.servo.get("linearLift");

    }
    public double linearLiftMovement() {

        //Linear Lift movement
        if (gamepad1.dpad_up) {
            liftPosition += LIFT_DELTA;
        } else if (gamepad1.dpad_down) {
            liftPosition -= LIFT_DELTA;
        }
        liftPosition = Range.clip(liftPosition, LINEAR_LIFT_DOWN, LINEAR_LIFT_UP);
        return liftPosition;
    }
    public void loop() {


        linearLift.setPosition(linearLiftMovement());
    }
}
