
package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class Teleop extends HelperOpMode {

    //Motor object variables
    DcMotor motorRight;
    DcMotor motorLeft;
    //DcMotor collector;
    //DcMotor scorer;
    //DcMotor lift;
    DcMotor winch1;
    DcMotor winch2;

    //Servo object variables
    //Servo linearLift;

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

        motorRight = hardwareMap.dcMotor.get("motor_Right");
        motorLeft = hardwareMap.dcMotor.get("motor_Left");
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        //collector = hardwareMap.dcMotor.get("collector");
        //scorer = hardwareMap.dcMotor.get("scorer");
        //lift = hardwareMap.dcMotor.get("lift");
        winch1 = hardwareMap.dcMotor.get("wench1");
        winch2 = hardwareMap.dcMotor.get("wench2");

        //linearLift = hardwareMap.servo.get("linearLift");
    }

    public double scorerMovement(){
        double currentValue = 0;

        //Scorer movement
        if (gamepad1.b){
            currentValue = FORWARD_SCORER_POWER;
        }
        else if (gamepad1.x)
        {
            currentValue = BACKWARD_SCORER_POWER;
        }
        return currentValue;
    }

    public double collectorMovement(){
        double currentValue = 0;

        //Collector movement
        if (gamepad1.right_bumper) {
            currentValue = FORWARD_COLLECTOR_POWER;
        }
        else if (gamepad1.left_bumper) {
            currentValue = BACKWARD_COLLECTOR_POWER;
        }
        return currentValue;
    }

    public double linearLiftMovement() {
        double currentValue = 0;

        //Linear Lift movement
        if (gamepad1.dpad_up) {
            currentValue = LINEAR_LIFT_UP;
        } else if (gamepad1.dpad_down) {
            currentValue = LINEAR_LIFT_DOWN;
        }
        return currentValue;
    }

    public double liftMovement (){
        double currentValue = 0;

        //Lift movement
        if (gamepad1.y)
            currentValue = UP_LIFT_POWER;
        else if (gamepad1.a)
            currentValue = DOWN_LIFT_POWER;
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

        //scorer.setPower(scorerMovement());
        //collector.setPower(collectorMovement());
        //linearLift.setPosition(linearLiftMovement());
        winch1.setPower(scaleInput(gamepad1.right_trigger));
        winch2.setPower(scaleInput(gamepad1.right_trigger));
        winch1.setPower(scaleInput(gamepad1.left_trigger)*-1);
        winch2.setPower(scaleInput(gamepad1.left_trigger)*-1);
        //lift.setPower(liftMovement());
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
