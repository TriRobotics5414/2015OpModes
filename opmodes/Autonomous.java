package com.qualcomm.ftcrobotcontroller.opmodes;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

import java.text.DecimalFormat;

/**
 * Created by MthSci.Student on 12/21/2015.
 */
public class Autonomous extends HelperOpMode{
    //Motor Objects
    DcMotor right;
    DcMotor left;

    //Encoder Move Distances
    int STARTING_MOVE = 101;
    int MOVE_TO_BASKET = 30;
    int MOVE_FROM_BASKET = 1;
    //int MOVE_TO_RAMP = 5;
    int MOVE_TO_PARK = 21;
    final int circumference = 33;

    //Gyro angles
    double FORTY_FIVE = 45.0;
    double NINETY = 90.0;

    //State variables
    boolean firstTime = true;
    boolean lastTime = false;
    int state = 1;

    //Navx variables
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
    DeviceInterfaceModule cdim;

    navXPIDController.PIDResult yawPIDResult;

    public Autonomous(){

    }

    public void init (){
        right = setupMotor("Right",true);
        left = setupMotor("Left",false);
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

    }

    public int convertDistanceToTicks(int distance){
        return (1120/circumference)*distance;
    }

    public void resetState(double angle){
        navx_device.zeroYaw();
        yawPIDResult = new navXPIDController.PIDResult();
        yawPIDController.setSetpoint(angle);
        resetEncoders(left);
        resetEncoders(right);
        firstTime = true;
        lastTime = false;

    }

    public void driveStraight (int distance, double speed){
        if(firstTime) {

            setupEncoders(right, false);
            setupEncoders(left, false);

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
        
        if(right.getCurrentPosition() >= distance){
            right.setPower(0.0);
            left.setPower(0.0);
            lastTime = true;
        }
    }

    public double limit(double a) {
        return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
    }

    public void turn(boolean direction){
        if ( yawPIDController.isNewUpdateAvailable(yawPIDResult) ) {
            if ( yawPIDResult.isOnTarget() ) {
                left.setPower(0.0);
                right.setPower(0.0);
                lastTime = true;
                telemetry.addData("Motor Output", df.format(0.00));
            } else {
                double output = yawPIDResult.getOutput();
                if (direction) {
                    left.setPower(output);
                    right.setPower(-output);
                    telemetry.addData("Motor Output", df.format(output) + ", " +
                            df.format(-output));
                }
                else {
                    left.setPower(-output);
                    right.setPower(output);
                    telemetry.addData("Motor Output", df.format(-output) + ", " +
                            df.format(output));

                }
            }
        }
    }

    public void loop (){
        switch(state){
            case 1:
                if (firstTime){
                    resetState(0.0);
                }
                driveStraight(convertDistanceToTicks(12), .3);
                if (lastTime){
                    state = 7;
                }
                break;
            case 2:
                resetState(45.0);
                turn(true);
                if (lastTime) {
                    state++;
                }
                break;

            case 3:
                resetState(0.0);
                driveStraight(convertDistanceToTicks(MOVE_TO_BASKET), .5);
                if (lastTime) {
                    state++;
                }
                break;

            case 4:
                resetState(0.0);
                driveStraight(convertDistanceToTicks(MOVE_FROM_BASKET), -.5);
                if (lastTime) {
                    state++;
                }
                break;

            case 5:
                resetState(90.0);
                turn(true);
                if (lastTime) {
                    state++;
                }
                break;

            case 6:
                resetState(0.0);
                driveStraight(convertDistanceToTicks(MOVE_TO_PARK), .5);
                if (lastTime) {
                    state++;
                }
                break;

            case 7:
                left.setPower(0.0);
                right.setPower(0.0);
                System.currentTimeMillis();

        }
        telemetry.addData("Current Position: " + right.getCurrentPosition(), "Target Position: " + right.getTargetPosition());
        telemetry.addData("Current Angle" + navx_device.getYaw(),"Target Angle" + yawPIDController.getSetpoint());
    }
}

