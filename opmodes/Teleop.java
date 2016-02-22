
package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class Teleop extends HelperOpMode {


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
        super.init();
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
        if (gamepad1.dpad_up) {
            liftPosition += LIFT_DELTA;
        } else if (gamepad1.dpad_down) {
            liftPosition -= LIFT_DELTA;
        }
        liftPosition = Range.clip(liftPosition, LINEAR_LIFT_DOWN, LINEAR_LIFT_UP);
        return liftPosition;
    }
    public double linearBlockMovement() {

        if (gamepad2.dpad_right) {
            linearBlockPosition += LIFT_DELTA;
        } else if (gamepad2.dpad_left) {
            linearBlockPosition -= LIFT_DELTA;
        }
        linearBlockPosition = Range.clip(linearBlockPosition, LINEAR_LIFT_DOWN, LINEAR_LIFT_UP);
        return linearBlockPosition;

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


    public double climberLeftMovement (){
        double currentValue = climberServoLeft.getPosition();

        if (gamepad1.dpad_left)
            currentValue = LEFT_CLIMBER_UP;
        else if (gamepad1.dpad_right)
            currentValue = LEFT_CLIMBER_DOWN;
        else if (gamepad1.start){
            currentValue = LEFT_CLIMBER_MID;
        }
        return currentValue;
    }
    public double climberRightMovement () {
        double currentValue = climberServoRight.getPosition();

        if (gamepad1.left_stick_button) {
            currentValue = RIGHT_CLIMBER_UP;
        }
        else if (gamepad1.right_stick_button) {
            currentValue = RIGHT_CLIMBER_DOWN;
        }
        else if (gamepad1.b) {
            currentValue = RIGHT_CLIMBER_MID;
        }
        return currentValue;
    }

    public double churroMovement(){
        double currentValue = churro.getPosition();
        if (gamepad2.dpad_up)
            currentValue = CHURRO_UP;
        else if (gamepad2.dpad_down)
            currentValue = CHURRO_DOWN;
        return currentValue;

    }

    public double winchMovement(){
        double currentValue = 0;

        if (gamepad2.left_trigger > 0.3)
            currentValue = gamepad2.left_trigger * -1;
        else if (gamepad2.right_trigger > 0.3)
            currentValue = gamepad2.right_trigger;
        return currentValue;
    }
public double climberRotate(){
    double currentValue = climberRotate.getPosition();
    if (gamepad1.left_bumper) {
         currentValue = ROTATE_IN;
    } else if (gamepad1.right_bumper) {
        currentValue = ROTATE_OUT;
    }
    return currentValue;
}

    public double climberElbow() {
        double currentValue = climberElbow.getPosition();
        if (gamepad1.right_trigger > 0.2) {
            currentValue = ELBOW_UP;
        } else if (gamepad1.left_trigger > 0.2) {
            currentValue = ELBOW_DOWN;
        }
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
        winch.setPower(winchMovement());
        lift.setPower(liftMovement());
        spinner.setPower(collectorMovement());

        linearLift.setPosition(linearLiftMovement());
        climberServoLeft.setPosition(climberLeftMovement());
        climberServoRight.setPosition(climberRightMovement());
        churro.setPosition(churroMovement());
        climberRotate.setPosition(climberRotate());
        climberElbow.setPosition(climberElbow());
        linearBlock.setPosition(linearBlockMovement());



        telemetry.addData("climberServoLeft: " + climberServoLeft.getPosition(), "");
        telemetry.addData("climberServoRight: " + climberServoRight.getPosition(), "");
        telemetry.addData("colorServoRight: " + colorServoRight.getPosition(), "");
        telemetry.addData("colorServoLeft: " + colorServoLeft.getPosition(), "");



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
