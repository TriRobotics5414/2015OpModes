package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogOutput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

//------------------------------------------------------------------------------
//
// PushBotHardware
//

/**
 * 5414 Helper Class to abstract some of the creation of and usage of hardware objects
 */
public class HelperOpMode extends OpMode

{
    //Motor object variables
    DcMotor motorRight, motorLeft, spinner, lift, winch;

    //Servo object variables
    Servo linearLift, linearBlock, climberServoLeft, climberServoRight, churro, climberRotate, climberElbow, colorSensorRight, colorSensorLeft;


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
    double LEFT_CLIMBER_UP = 0.0;
    double LEFT_CLIMBER_DOWN = 1;
    double RIGHT_CLIMBER_UP = .9;
    double RIGHT_CLIMBER_DOWN = 0.0;
    double RIGHT_CLIMBER_MID = 0.5;
    double LEFT_CLIMBER_MID = 0.5;
    double CHURRO_UP = .1;
    double CHURRO_DOWN = .8;
    double LEFT_COLOR_UP = 0.0;
    double RIGHT_COLOR_UP = 0.9;
    double LEFT_COLOR_DOWN = .8;
    double RIGHT_COLOR_DOWN = .2;
    double ROTATE_IN = 0.7;
    double ROTATE_OUT = 0.4;
    double ELBOW_DOWN = 0.2;
    double ELBOW_UP = 1.0;

    ColorSensor sensorRGB;
    AnalogInput ir;
    OpticalDistanceSensor ods;

    //Encoder Move Distances
    int STARTING_MOVE = 110;
    int MOVE_TO_BASKET = 18;
    int MOVE_FROM_BASKET = 1;
    int MOVE_TO_RAMP = 5;
    int MOVE_TO_PARK = 21;
    final double circumference = 6.5;

    //Beacon variables
    double BUTTON_RIGHT = 1;
    double CLIMBER_ELBOW = .1;

    //Gyro angles
    double FORTY_FIVE = 45.0;
    double NINETY = 90.0;

    //State variables
    boolean firstTime = true;
    boolean lastTime = false;
    int state = 1;
    double navxReset = .05;

    double liftPosition = 0.0;
    double linearBlockPosition = 0.0;
    public HelperOpMode(){

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
        motorRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        spinner = hardwareMap.dcMotor.get("Spinner");
        spinner.setDirection(DcMotor.Direction.REVERSE);
        lift = hardwareMap.dcMotor.get("Lift");
        winch = hardwareMap.dcMotor.get("Winch");

        linearLift = hardwareMap.servo.get("linearLift");
        linearBlock = hardwareMap.servo.get("linearBlock");
        climberServoLeft = hardwareMap.servo.get("climberServoLeft");
        climberServoLeft.setDirection(Servo.Direction.REVERSE);
        climberServoRight = hardwareMap.servo.get("climberServoRight");
        climberServoRight.setPosition(RIGHT_CLIMBER_UP);
        churro = hardwareMap.servo.get("churro");
        churro.setPosition(CHURRO_UP);
        climberRotate = hardwareMap.servo.get("climberRotate");
        climberRotate.setPosition(ROTATE_IN);
        climberElbow = hardwareMap.servo.get("climberElbow");
        climberElbow.setPosition(ELBOW_DOWN);
        colorSensorLeft = hardwareMap.servo.get("colorSensorLeft");
        colorSensorLeft.setPosition(LEFT_COLOR_UP);
        colorSensorRight = hardwareMap.servo.get("colorSensorRight");
        colorSensorRight.setPosition(RIGHT_COLOR_UP);

        ods = hardwareMap.opticalDistanceSensor.get("ods");
        ir = hardwareMap.analogInput.get("ir");
    }

    @Override
    public void loop(){

    }

    /**
     * Method to setup a motor
     * @param motorName name of the motor in the config
     * @param direction direction you want to (true == forward, false ==  backwards)
     * @return created motor
     */
    public DcMotor setupMotor(String motorName, boolean direction){
        DcMotor motor = hardwareMap.dcMotor.get(motorName);
        if(!direction) {
            motor.setDirection(DcMotor.Direction.REVERSE);
        }
        return motor;
    }

    /**
     * Method to reset the encoder values to 0
     * @param motor created motor object
     */
    public void resetEncoders(DcMotor motor){
        motor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    public void setupEncoders(DcMotor motor, boolean encoderType){
        if(encoderType){
            motor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
        else{
            motor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        }
    }
    public Servo setupServo (String servoName){
        Servo servo = hardwareMap.servo.get (servoName);
        return servo;
    }




}