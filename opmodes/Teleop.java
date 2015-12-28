
package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class Teleop extends HelperOpMode {

    //Motor object variables
    DcMotor motorRight, motorLeft, spinner, belts, lift, winch;

    //Servo object variables
    Servo linearLift, leftGate, rightGate, climberServo;


    final static double LIFT_DELTA = .01;

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

    double liftPosition = 0.2;



    /**
     * Constructor
     */
    public Teleop() {

    }


    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {

        motorRight = hardwareMap.dcMotor.get("Right");
        motorLeft = hardwareMap.dcMotor.get("Left");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        spinner = hardwareMap.dcMotor.get("Spinner");
        belts = hardwareMap.dcMotor.get("Belts");
        lift = hardwareMap.dcMotor.get("Lift");
        winch = hardwareMap.dcMotor.get("Winch");

        linearLift = hardwareMap.servo.get("linearLift");
        leftGate = hardwareMap.servo.get("leftGate");
        rightGate = hardwareMap.servo.get("rightGate");
        climberServo = hardwareMap.servo.get("climberServo");
    }

    public double scorerMovement(){
        double currentValue = 0;

        //Scorer movement
        if (gamepad2.b){
            currentValue = FORWARD_SCORER_POWER;
        }
        else if (gamepad2.x)
        {
            currentValue = BACKWARD_SCORER_POWER;
        }
        return currentValue;
    }

    public double collectorMovement(){
        double currentValue = 0;

        //Collector movement
        if (gamepad2.right_bumper) {
            currentValue = FORWARD_COLLECTOR_POWER;
        }
        else if (gamepad2.left_bumper) {
            currentValue = BACKWARD_COLLECTOR_POWER;
        }
        return currentValue;
    }

    public double linearLiftMovement() {

        //Linear Lift movement
        if (gamepad2.dpad_up) {
            liftPosition += LIFT_DELTA;
        } else if (gamepad2.dpad_down) {
            liftPosition -= LIFT_DELTA;
        }
        liftPosition = Range.clip(liftPosition, LINEAR_LIFT_DOWN, LINEAR_LIFT_UP);
        return liftPosition;
    }
    public double leftGateMovement() {
        double currentValue = leftGate.getPosition();

        //Linear Lift movement
        if (gamepad2.dpad_left) {
            currentValue = LEFT_GATE_DOWN;
        } else {
            currentValue = LEFT_GATE_UP;
        }
        return currentValue;
    }
    public double rightGateMovement() {
        double currentValue = rightGate.getPosition();

        //Linear Lift movement
        if (gamepad2.dpad_right) {
            currentValue = RIGHT_GATE_UP;
        } else {
            currentValue = RIGHT_GATE_DOWN;
        }
        return currentValue;
    }
    public double liftMovement () {
        double currentValue = 0;

       //Lift movement
        if (gamepad2.y)
            currentValue = UP_LIFT_POWER;
        else if (gamepad2.a)
            currentValue = DOWN_LIFT_POWER;
        return currentValue;
    }
    public double climberMovement (){
        double currentValue = climberServo.getPosition();

        if (gamepad1.dpad_up)
            currentValue = RIGHT_GATE_UP;
        else if (gamepad1.dpad_down)
            currentValue = RIGHT_GATE_DOWN;
        return currentValue;
    }
    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

        //Drivetrain movement
        motorLeft.setPower(scaleInput(gamepad1.left_stick_y));
        motorRight.setPower(scaleInput(gamepad1.right_stick_y));

        belts.setPower(scorerMovement());
        spinner.setPower(collectorMovement());
        linearLift.setPosition(linearLiftMovement());
        leftGate.setPosition(leftGateMovement());
        rightGate.setPosition(rightGateMovement());
        climberServo.setPosition(climberMovement());

        if (gamepad2.right_trigger != 0) {
            winch.setPower(scaleInput(gamepad2.right_trigger));
        }
        else {
            winch.setPower(scaleInput(gamepad2.left_trigger) * -1);
        }
        lift.setPower(liftMovement());
        telemetry.addData("Left Motor: " + motorLeft.getPower(), "Right Motor" + motorRight.getPower());
        telemetry.addData("Belts: " + belts.getPower(), "Spinner" + spinner.getPower());
        telemetry.addData("Left Gate: " + leftGate.getPosition(),"Right Gate: "+ rightGate.getPosition());

    }



    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}
