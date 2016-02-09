/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
package com.qualcomm.ftcrobotcontroller.opmodes;

import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;

/*
 * An example loop op mode where the robot will drive in
 * a straight line (where the driving direction is guided by
 * the Yaw angle from a navX-Model device).
 *
 * This example uses a simple PID controller configuration
 * with a P coefficient, and will likely need tuning in order
 * to achieve optimal performance.
 *
 * Note that for the best accuracy, a reasonably high update rate
 * for the navX-Model sensor should be used.  This example uses
 * the default update rate (50Hz), which may be lowered in order
 * to reduce the frequency of the updates to the drive system.
 */

public class SensorTest extends OpMode {
    DcMotor idk, left;
    OpticalDistanceSensor ods;
    AnalogInput sonar;
    //ColorSensor colorSensor;
    private AHRS navx_device;
    private navXPIDController yawPIDController;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    private final double TARGET_ANGLE_DEGREES = 90.0;
    private final double TOLERANCE_DEGREES = .5;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;
    private final int NAVX_DIM_I2C_PORT = 0;
    DecimalFormat df;
    boolean completed = false;
    DeviceInterfaceModule cdim;

    navXPIDController.PIDResult yawPIDResult;


    /* This is the port on the Core Device Interface Module        */
    /* in which the navX-Model Device is connected.  Modify this  */
    /* depending upon which I2C port you are using.               */

    public void start()
    {
        navx_device.zeroYaw();
        yawPIDResult = new navXPIDController.PIDResult();
    }
    public void navxReset(double angle){
        navx_device.zeroYaw();
        yawPIDController.setSetpoint(angle);

    }
    @Override
    public void init() {
        sonar = hardwareMap.analogInput.get("sonar");
        cdim = hardwareMap.deviceInterfaceModule.get("DIM");
        //cdim.setDigitalChannelMode(5, DigitalChannelController.Mode.OUTPUT);
        /*idk = hardwareMap.dcMotor.get("idk");
        ods = hardwareMap.opticalDistanceSensor.get("ods");
        //colorSensor = hardwareMap.colorSensor.get("color");
        left = hardwareMap.dcMotor.get("left");*/

        /* If possible, use encoders when driving, as it results in more */
        /* predictable drive system response.                           */
        //leftMotor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //rightMotor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);


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


        //leftMotor = hardwareMap.dcMotor.get("left_drive");
        //rightMotor = hardwareMap.dcMotor.get("right_drive");

    }
    /*public void colorTest(){

        if((colorSensor.red())>(colorSensor.blue())){
            idk.setPower(0.0);
        }
        else if((colorSensor.blue())>(colorSensor.red())){
            idk.setPower(1.0);
        }
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());

    }*//*

    public void turnTest(){
        double drive_speed = 0.5;

        if ( yawPIDController.isNewUpdateAvailable(yawPIDResult) ) {
            if ( yawPIDResult.isOnTarget() ) {
                idk.setPower(drive_speed);
                telemetry.addData("Motor Output",df.format(drive_speed) + ", " +
                        df.format(drive_speed));
            } else {
                double output = yawPIDResult.getOutput();
                idk.setPower(limit(drive_speed + output));
                telemetry.addData("Motor Output", df.format(limit(drive_speed + output)) + ", " +
                        df.format(limit(drive_speed - output)));
            }
        } else {
            *//* No sensor update has been received since the last time  *//*
            *//* the loop() function was invoked.  Therefore, there's no *//*
            *//* need to update the motors at this time.                 *//*
        }
        telemetry.addData("Yaw", df.format(navx_device.getYaw()));
    }

    public void turnTest1(){
        double drive_speed = 0.5;

        if ( yawPIDController.isNewUpdateAvailable(yawPIDResult) ) {
            if ( yawPIDResult.isOnTarget() ) {
                drive_speed = 0;
                idk.setPower(drive_speed);
                completed = true;
                telemetry.addData("Motor Output", df.format(drive_speed) + ", " +
                        df.format(drive_speed));
                navxReset(-90.0);
            } else {
                double output = yawPIDResult.getOutput();
                idk.setPower(drive_speed);
                telemetry.addData("Motor Output", df.format(limit(drive_speed + output)) + ", " +
                        df.format(limit(drive_speed - output)));
            }
        } else {
            *//* No sensor update has been received since the last time  *//*
            *//* the loop() function was invoked.  Therefore, there's no *//*
            *//* need to update the motors at this time.                 *//*
        }
        telemetry.addData("Yaw", df.format(navx_device.getYaw()));
    }

    public void odsSensorTest(){
        if(ods.getLightDetected() > 0.28){
            left.setPower(0.0);
        }
        else {
            left.setPower(1.0);
        }
        telemetry.addData("ods: ", ods.getLightDetected());
    }

    public void turnTest2(){
        double drive_speed = 0.5;

        if ( yawPIDController.isNewUpdateAvailable(yawPIDResult) ) {
            if ( !yawPIDResult.isOnTarget() ) {
                drive_speed = 0;
                idk.setPower(drive_speed);

                telemetry.addData("Motor Output", df.format(drive_speed) + ", " +
                        df.format(drive_speed));

            } else {
                double output = yawPIDResult.getOutput();
                idk.setPower(drive_speed);
                telemetry.addData("Motor Output", df.format(limit(drive_speed + output)) + ", " +
                        df.format(limit(drive_speed - output)));
            }
        } else {
            *//* No sensor update has been received since the last time  *//*
            *//* the loop() function was invoked.  Therefore, there's no *//*
            *//* need to update the motors at this time.                 *//*
        }
        telemetry.addData("Yaw", df.format(navx_device.getYaw()));
    }

    public double limit(double a) {
        return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
    }*/
    @Override
    public void loop() {
      telemetry.addData("sensor output", sonar.getValue());



    }
    @Override
    public void stop() {

    }
}
