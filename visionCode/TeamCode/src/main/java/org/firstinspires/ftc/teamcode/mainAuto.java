/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@Autonomous(name="Ramp Auto", group="Linear Opmode") // @Autonomous(...) is the other common choice
public class mainAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer1 = new ElapsedTime();
    ElapsedTime ballTime = new ElapsedTime();
    DcMotor motorLeft,
            motorRight, intakeMotor;
    ColorSensor colorSensor1;
    DeviceInterfaceModule deviceInterface1;
    DcMotorController motorController1,
            motorController2;
    OpticalDistanceSensor distanceSensor1;
    double rightError,
        leftError,
        numberOfCycles;
    int originalFlyWheelEncoderCount = 0;
    int step;
    boolean isBangBangFinished,
        isRobotFlush;
    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable
    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read
    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 60;
    private final double TOLERANCE_DEGREES = 2;
    private final double MIN_MOTOR_OUTPUT_VALUE = -0.4;
    private final double MAX_MOTOR_OUTPUT_VALUE = 0.4;
    private final double YAW_PID_P = .5;
    private final double YAW_PID_I = 0;
    private final double YAW_PID_D = 0;
    private boolean calibration_complete = false;
    private final int NAVX_DIM_I2C_PORT = 0;
    public final double encoderCountsPerInch = 70.736;
    private AHRS navx_device;
    private navXPIDController yawPIDController;
    int distanceSensorSonar;
    int distanceSensorODS;
    boolean isTurnFinished;
    boolean isRightFinished,
            isLeftFinished;
    @Override
    public void runOpMode() {
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //flywheelMotor = hardwareMap.dcMotor.get("flywheelMotor");
        //flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        motorController1 = hardwareMap.dcMotorController.get("motorController1");
        motorController2 = hardwareMap.dcMotorController.get("motorController2");
        deviceInterface1 = hardwareMap.deviceInterfaceModule.get("interfaceModule1");
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        colorSensor1 = hardwareMap.colorSensor.get("colorSensor1");
        distanceSensor1 = hardwareMap.opticalDistanceSensor.get("distanceSensor1");
        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();
        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("interfaceModule1"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        while (motorLeft.getCurrentPosition() != 0 || motorRight.getCurrentPosition() != 0 ) {

        }

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(motorLeft.getMode() != DcMotor.RunMode.RUN_USING_ENCODER || motorRight.getMode() != DcMotor.RunMode.RUN_USING_ENCODER ){
            telemetry.addData("Left Motor Mode: ", motorLeft.getMode());
            telemetry.addData("Right Motor Mode: ", motorRight.getMode());
           // telemetry.addData("Fly Wheel Motor Mode: ", flywheelMotor.getMode());
            telemetry.update();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
           /* Create a PID Controller which uses the Yaw Angle as input. */
        yawPIDController = new navXPIDController( navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);

        while ( !calibration_complete ) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            calibration_complete = !navx_device.isCalibrating();
            if (!calibration_complete) {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
                telemetry.update();
            }
        }
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("navX-Micro", "Startup Calibration Finished");
        telemetry.update();
        yawPIDController.enable(true);
        navx_device.zeroYaw();
        waitForStart();
        runtime.reset();
        numberOfCycles = 0;
        step = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            distanceSensorSonar = (range1Cache[0] & 0xFF);
            distanceSensorODS = (range1Cache[1] & 0xFF);
            telemetry.addData("Get the Gyro", navx_device.getYaw());
            telemetry.addData("Right Error", rightError);
            telemetry.addData("Left Error", leftError);
            telemetry.addData("Left Pos", motorLeft.getCurrentPosition());
            telemetry.addData("Right Pos", motorRight.getCurrentPosition());
            telemetry.update();

            //Actual actions go here.
                turnToAngle(90);
            /*
            if((range1Cache[0] & 0xFF) > 50){
                setPower(1);
            } else if((range1Cache[0] & 0xFF) < 50 && (range1Cache[0] & 0xFF) > 10){
                setPower(0.5);
            } else if((range1Cache[0] & 0xFF) < 10){
                ballTime.reset();
                ballTime.startTime();
                if(ballTime.seconds() < 1){
                    setPower(1);
                } else {
                    setPower(0);
                }
            }
         */

            numberOfCycles += 1;
        }
    }

    /*public boolean isRed(){
        if(colorSensor1.red() <= 6 && colorSensor1.red() >= 3 && !isBlue()) {
            return true;
        }
        return false;
    }
    public boolean isBlue(){
        if(colorSensor1.blue() <= 6 && colorSensor1.blue() >= 3 && !isRed()){
            return true;
        }
        return false;
    }*/
    public void BangBangLoopDrive(double target) {
        isBangBangFinished = false;
        isLeftFinished = false;
        isRightFinished = false;
        target = target * encoderCountsPerInch;
        rightError = target - motorRight.getCurrentPosition();
        leftError = target - motorLeft.getCurrentPosition();
        //Get the derivative of the error

        if(leftError > 850){
            motorLeft.setPower(1);
        }
        if(leftError < -850){
            motorLeft.setPower(-1);
        }
        else if(leftError > -850 && leftError <= -600){
            motorLeft.setPower(-.5);
        }
        else if(leftError > -600 && leftError <= -450){
            motorLeft.setPower(-.4);
        }
        else if(leftError > -450 && leftError <= -350){
            motorLeft.setPower(-.3);
        }
        else if(leftError > -350 && leftError <= -250){
            motorLeft.setPower(-.2);
        } else if(leftError > -50 && leftError < 0){
            isLeftFinished = true;
        }
////////////
        else if(leftError < 850 && leftError >= 600){
            motorLeft.setPower(.5);

        }
        else if(leftError < 600 && leftError >= 450){
            motorLeft.setPower(.4);

        }
        else if(leftError < 450 && leftError >= 350){
            motorLeft.setPower(.3);

        }
        else if(leftError < 350 && leftError >= 250){
            motorLeft.setPower(.2);
        } else if(leftError < 50 && leftError >= 0){
            isLeftFinished = true;
        }

        if(rightError > 850){
            motorRight.setPower(1);
        }
        if(rightError < -850){
            motorRight.setPower(-1);
        }
        else if(rightError > -850 && rightError <= -600){
            motorRight.setPower(-.5);
        }
        else if(rightError > -600 && rightError <= -450){
            motorRight.setPower(-.4);
        }
        else if(rightError > -450 && rightError <= -350){
            motorRight.setPower(-.3);
        }
        else if(rightError > -350 && rightError <= -250){
            motorRight.setPower(-.2);
        } else if(rightError > -50 && rightError < 0){
            isRightFinished = true;
        }
////////////
        else if(rightError < 850 && rightError >= 600){
            motorRight.setPower(.5);

        }
        else if(rightError < 600 && rightError >= 450){
            motorRight.setPower(.4);

        }
        else if(rightError < 450 && rightError >= 350){
            motorRight.setPower(.3);

        }
        else if(rightError < 350 && rightError >= 250){
            motorRight.setPower(.2);

        } else if(rightError >= 0 && rightError < 50){
            isRightFinished = true;
        }

        if(isRightFinished == true && isLeftFinished == true){
            isBangBangFinished = true;
            motorRight.setPower(0);
            motorLeft.setPower(0);
        } else {
            isBangBangFinished = false;
        }
    }
    public void turnToAngle(double angle){
         isTurnFinished = false;
        yawPIDController.setSetpoint(angle);
        navx_device.zeroYaw();
        try {
            yawPIDController.enable(true);

        /* Wait for new Yaw PID output values, then update the motors
           with the new PID value with each new output value.
         */

            final double TOTAL_RUN_TIME_SECONDS = 30;
            int DEVICE_TIMEOUT_MS = 500;
            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

            DecimalFormat df = new DecimalFormat("#.##");

            while ( (runtime.time() < TOTAL_RUN_TIME_SECONDS) &&
                    !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        motorLeft.setPowerFloat();
                        motorRight.setPowerFloat();
                        telemetry.addData("PIDOutput", df.format(0.00));
                        telemetry.update();
                    } else {
                        double output = yawPIDResult.getOutput();
                        motorLeft.setPower(-output);
                        motorRight.setPower(output );
                        telemetry.addData("Output", output);
                        telemetry.addData("PIDOutput", df.format(output) + ", " +
                                df.format(-output));
                        telemetry.update();
                    }
                } else {
			    /* A timeout occurred */
                    Log.w("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
                telemetry.addData("Yaw", df.format(navx_device.getYaw()));
                telemetry.update();
            }
        }
        catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        finally {
            navx_device.close();
            telemetry.addData("LinearOp", "Complete");
            telemetry.update();
        }
        isTurnFinished = true;
    }
    public double limit(double a) {
        return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
    }
    public void driveStraight(double drive_speed){
        navx_device.zeroYaw();

        /* Wait for new Yaw PID output values, then update the motors
           with the new PID value with each new output value.
         */

        final double TOTAL_RUN_TIME_SECONDS = 10.0;
        int DEVICE_TIMEOUT_MS = 500;
        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();
        DecimalFormat df = new DecimalFormat("#.##");
        try {
            while ((runtime.time() < TOTAL_RUN_TIME_SECONDS) &&
                    !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        motorLeft.setPower(drive_speed );
                        motorRight.setPower(drive_speed);
                        telemetry.addData("PIDOutput", df.format(drive_speed) + ", " +
                                df.format(drive_speed));
                    } else {
                        double output = yawPIDResult.getOutput();
                        motorLeft.setPower(drive_speed + output);
                        motorRight.setPower(drive_speed - output);
                        telemetry.addData("PIDOutput", df.format(limit(drive_speed + output)) + ", " +
                                df.format(limit(drive_speed - output)));
                    }
                    telemetry.addData("Yaw", df.format(navx_device.getYaw()));
                } else{
			        /* A timeout occurred */
                    Log.w("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
            }
        }
        catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        finally {
            navx_device.close();
            telemetry.addData("LinearOp", "Complete");
        }
    }
    public void driveToWall(){
        isRobotFlush = false;
        numberOfCycles = 0;
        timer1.reset();
        if(distanceSensorSonar >= 40){
            setPower(0.5);
            isRobotFlush = false;
        } else if(distanceSensorSonar >= 9 && distanceSensorSonar < 40){
            setPower(0.2);
            isRobotFlush = false;
        } else if(distanceSensorSonar < 9){
            timer1.startTime();
            while(timer1.seconds() < 0.5){
                motorLeft.setPower(0);
                motorRight.setPower(.25);
                isRobotFlush = false;
            } while(timer1.seconds() >= 0.5 && timer1.seconds() < 1){
                motorRight.setPower(0);
                motorLeft.setPower(.25);
                isRobotFlush = false;
            }
                isRobotFlush = true;
        }
    }
    public void setPower(double power){
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }
    public void lineFollower(){
        double threshold = (0.06 + 0.3) / 2;
        if(distanceSensor1.getLightDetected() < threshold){
            motorLeft.setPower(-0.78);
            motorRight.setPower(-0.25);

        } else {
            motorLeft.setPower(-0.25);
            motorRight.setPower(-0.78);
        }
    }
}

          /*  telemetry.addData("Current Right Error: ", oldErrorRight);
            telemetry.addData("Current Left Error: ", oldErrorLeft);
            telemetry.addData("ODS: ", distanceSensorODS);
            telemetry.addData("Sonar Distance: ", distanceSensorSonar);
            telemetry.addData("Right Error Rate of Change: ", rateOfChangeDrive()); */