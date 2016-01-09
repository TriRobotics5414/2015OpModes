package com.qualcomm.ftcrobotcontroller.opmodes;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;

import java.text.DecimalFormat;

/**
 * Created by MthSci.Student on 12/21/2015.
 */
public class Autonomous extends HelperOpMode{
    //Motor Objects
    DcMotor right, left;

    Servo buttonRight, buttonLeft, climberElbow, linearLift, climberRotate, climberServoLeft, climberServoRight;

    ColorSensor sensorRGB;

    //Encoder Move Distances
    int STARTING_MOVE = 101;
    int MOVE_TO_BASKET = 24;
    int MOVE_FROM_BASKET = 1;
    //int MOVE_TO_RAMP = 5;
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
    //Navx variables
    private AHRS navx_device;
    private navXPIDController yawPIDController;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    private final double TARGET_ANGLE_DEGREES = 0;
    private final double TOLERANCE_DEGREES = .5;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;
    private final int NAVX_DIM_I2C_PORT = 4;
    DecimalFormat df;
    DeviceInterfaceModule cdim;

    navXPIDController.PIDResult yawPIDResult;

    public Autonomous(){

    }

    public void init (){
        right = setupMotor("Right", false);
        left = setupMotor("Left", true);

        climberServoLeft = hardwareMap.servo.get("climberServoLeft");
        climberServoRight = hardwareMap.servo.get("climberServoRight");
        climberServoLeft.setPosition(0.0);
        climberServoRight.setPosition(0.9);
        buttonRight = hardwareMap.servo.get("buttonRight");
        buttonLeft = hardwareMap.servo.get("buttonLeft");
        buttonLeft.setPosition(0.0);
        buttonRight.setPosition(0.0);
        linearLift = hardwareMap.servo.get("linearLift");
        linearLift.setPosition(0.0);
        climberRotate = hardwareMap.servo.get("climberRotate");
        climberRotate.setPosition(0.0);
        climberElbow = hardwareMap.servo.get("climberElbow");
        climberElbow.setPosition(1.0);

        // get a reference to our ColorSensor object.
        sensorRGB = hardwareMap.colorSensor.get("color");

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("DIM"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);
        /* Create a PID Controller which uses the Yaw Angle as input. */
        yawPIDController = new navXPIDController( navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);
        df = new DecimalFormat("#.##");
        resetEncoders();
    }

    @Override
    public void start() {
        /* reset the navX-Model device yaw angle so that whatever direction */
        /* it is currently pointing will be zero degrees.                   */
        navx_device.zeroYaw();
        yawPIDResult = new navXPIDController.PIDResult();
    }

    public int convertDistanceToTicks(int distance){
        return (int)((1120/circumference)*distance);
    }

    public void resetState(){
        navx_device.zeroYaw();
        yawPIDResult = new navXPIDController.PIDResult();

        firstTime = true;
        lastTime = false;

    }

    public void resetNavx(){
        navx_device.zeroYaw();
        yawPIDResult = new navXPIDController.PIDResult();
    }

    public void resetEncoders (){
        resetEncoders(left);
        resetEncoders(right);
        resetNavx();
    }

    public void driveStraight (int distance, double speed){
        if(firstTime) {

            setupEncoders(right, true);
            setupEncoders(left, true);

            right.setTargetPosition(distance);
            left.setTargetPosition(distance);

            right.setPower(speed);
            left.setPower(speed);
            firstTime = false;
        }
        if(!lastTime) {
            if (yawPIDController.isNewUpdateAvailable(yawPIDResult)) {
                if (yawPIDResult.isOnTarget()) {
                    left.setPower(speed);
                    right.setPower(speed);
                    telemetry.addData("Motor Output", df.format(speed) + ", " +
                            df.format(speed));
                } else {
                    double output = yawPIDResult.getOutput();
                    left.setPower(limit(speed + output));
                    right.setPower(limit(speed - output));
                    telemetry.addData("Motor Output", df.format(limit(speed + output)) + ", " +
                            df.format(limit(speed - output)));
                }
            }
        }
        
        if(Math.abs(right.getCurrentPosition()) >= distance){
            right.setPower(0.0);
            left.setPower(0.0);
            lastTime = true;
        }
    }

    public double limit(double a) {
        return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
    }

    public void beaconScoring(){



            climberElbow.setPosition(CLIMBER_ELBOW);

            /*if (sensorRGB.blue() > 355) {
                buttonRight.setPosition(BUTTON_RIGHT);
            } else if (sensorRGB.blue() < 355) {
                buttonLeft.setPosition(BUTTON_RIGHT);
            }

            if (buttonRight.getPosition() == BUTTON_RIGHT && climberElbow.getPosition() == CLIMBER_ELBOW) {
                lastTime = true;
            }
            telemetry.addData("Red ", sensorRGB.red());
            telemetry.addData("Blue ", sensorRGB.blue());*/


    }


    public void turn(boolean direction, double angle){
        if (firstTime) {
            setupEncoders(left, true);
            setupEncoders(right, true);
            if (direction) {
                left.setPower(.15);
                right.setPower(-.15);
            } else {
                left.setPower(-0.15);
                right.setPower(0.15);
            }
            firstTime = false;
        }
        if (navx_device.getYaw()> angle) {
            left.setPower(0.0);
            right.setPower(0.0);
            lastTime = true;
        }

    }

    public void stop (){
        resetEncoders();
    }

    public void loop (){

        switch(state){
            case 1:
                driveStraight(convertDistanceToTicks(STARTING_MOVE), -.3);
                climberRotate.setPosition(.3);
                if (lastTime){
                    resetEncoders();
                    resetNavx();
                    if (right.getCurrentPosition()== 0 && Math.abs(navx_device.getYaw()) < navxReset ){
                        resetState();
                        state++;
                    }
                }
                break;
            case 2:
                turn(true, 40.0);
                if (lastTime) {
                    resetEncoders();
                    if (right.getCurrentPosition()== 0 && Math.abs(navx_device.getYaw()) < navxReset ){
                        resetState();
                        state++;
                    }
                }
                break;

            case 3:
                driveStraight(convertDistanceToTicks(MOVE_TO_BASKET), -.3);
                if (lastTime) {
                    resetEncoders();
                    if (right.getCurrentPosition()== 0 && Math.abs(navx_device.getYaw()) <navxReset ){
                        state++;
                    }
                }
                break;

            case 4:
                beaconScoring();
                break;

            case 5:
                if (firstTime){
                    resetState();
                }
                driveStraight(convertDistanceToTicks(MOVE_FROM_BASKET), .3);
                if (lastTime) {
                    resetEncoders();
                    if (right.getCurrentPosition()== 0 && Math.abs(navx_device.getYaw()) <navxReset ){
                        state++;
                    }
                }
                break;

            case 6:
                if (firstTime){
                    resetState();
                }
                turn(true, 90.0);
                if (lastTime) {
                    resetEncoders();
                    if (right.getCurrentPosition()== 0 && Math.abs(navx_device.getYaw()) <navxReset ){
                        state++;
                    }
                }
                break;

            case 7:
                if (firstTime){
                    resetState();
                }
                driveStraight(convertDistanceToTicks(MOVE_TO_PARK), -.3);
                if (lastTime) {
                    resetEncoders();
                    if (right.getCurrentPosition()== 0 && Math.abs(navx_device.getYaw()) <navxReset ){
                        state++;
                    }
                }
                break;

            case 8:
                /*left.setPower(0.0);
                right.setPower(0.0);*/
                System.currentTimeMillis();

        }
        //telemetry.clearData();
        telemetry.addData("state: ", state);
        telemetry.addData("Current Position: " + right.getCurrentPosition(), "Target Position: " + right.getTargetPosition());
        telemetry.addData("Current Position L: " + left.getCurrentPosition(), "Target Position: " + left.getTargetPosition());
        telemetry.addData("" + navx_device.getYaw(), "Target: " + yawPIDController.getSetpoint());

    }
}

