package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

//------------------------------------------------------------------------------
//
// PushBotHardware
//

/**
 * Provides a single hardware access point between custom op-modes and the
 * OpMode class for the Push Bot.
 *
 * This class prevents the custom op-mode from throwing an exception at runtime.
 * If any hardware fails to map, a warning will be shown via telemetry data,
 * calls to methods will fail, but will not cause the application to crash.
 *
 * @author SSI Robotics
 * @version 2015-08-13-20-04
 */
public class HelperOpMode extends OpMode

{
    public HelperOpMode(){

    }

    @Override
    public void init() {

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
        motor.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    public void setupEncoders(DcMotor motor, boolean encoderType){
        if(encoderType){
            motor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
        else{
            motor.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        }
    }
    public Servo setupServo (String servoName){
        Servo servo = hardwareMap.servo.get (servoName);
        return servo;
    }




}