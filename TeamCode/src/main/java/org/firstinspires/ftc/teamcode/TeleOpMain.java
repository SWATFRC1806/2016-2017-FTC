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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp Drive and Feedback", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class TeleOpMain extends OpMode
{
    DcMotor motorLeft,
            motorRight;

    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;

    private I2cDevice colorSensorLeft;
    private I2cDeviceSynch colorSensorLeftReader;
    private I2cDevice colorSensorRight;
    private I2cDeviceSynch colorSensorRightReader;
    private byte[] colorSensorLeftCache;
    private byte[] colorSensorRightCache;

    ElapsedTime BangBangDriveTime;
    Boolean BangBangDriveFinished = false;
    Boolean shouldBangBangRun = false;

    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;
    private navXPIDController yawPIDController;

    private ElapsedTime runtime = new ElapsedTime();

    private boolean calibration_complete = false;

    @Override
    public void init() {
        colorSensorLeft = hardwareMap.i2cDevice.get("colorSensorLeft");
        colorSensorLeftReader = new I2cDeviceSynchImpl(colorSensorLeft, I2cAddr.create8bit(0x3c), false);
        colorSensorLeftReader.engage();

        colorSensorRight = hardwareMap.i2cDevice.get("colorSensorRight");
        colorSensorRightReader = new I2cDeviceSynchImpl(colorSensorRight, I2cAddr.create8bit(0x4c), false);
        colorSensorRightReader.engage();

        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kQuatAndRawData);
        
        while(motorRight.getCurrentPosition() != 0 || motorLeft.getCurrentPosition() != 0){
            //wait for reset
        }

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(motorRight.getMode() !=  DcMotor.RunMode.RUN_USING_ENCODER || motorLeft.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
            //wait for mode change
        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("navX Op Init Loop", runtime.toString());
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        colorSensorRightCache = colorSensorRightReader.read(0x1c, 1);
        colorSensorLeftCache = colorSensorLeftReader.read(0x1c, 1);

        boolean connected = navx_device.isConnected();
        telemetry.addData("1 navX-Device", connected ? "Connected" : "Disconnected" );
        String gyrocal, gyro_raw, accel_raw, mag_raw;
        boolean magnetometer_calibrated;

        if ( connected ) {
            DecimalFormat df = new DecimalFormat("#.##");
            magnetometer_calibrated = navx_device.isMagnetometerCalibrated();
            gyro_raw = df.format(navx_device.getRawGyroX()) + ", " +
                    df.format(navx_device.getRawGyroY()) + ", " +
                    df.format(navx_device.getRawGyroZ());
            accel_raw = df.format(navx_device.getRawAccelX()) + ", " +
                    df.format(navx_device.getRawAccelY()) + ", " +
                    df.format(navx_device.getRawAccelZ());
            if ( magnetometer_calibrated ) {
                mag_raw = df.format(navx_device.getRawMagX()) + ", " +
                        df.format(navx_device.getRawMagY()) + ", " +
                        df.format(navx_device.getRawMagZ());
            } else {
                mag_raw = "Uncalibrated";
            }
        } else {
            gyro_raw =
                    accel_raw =
                            mag_raw = "-------";
        }
        telemetry.addData("2 Gyros (Degrees/Sec):", gyro_raw);
        telemetry.addData("3 Accelerometers  (G):", accel_raw );
        telemetry.addData("4 Magnetometers  (uT):", mag_raw );
        telemetry.addData("Current Heading: ", navx_device.getYaw());
        telemetry.addData("Color Sensor Left Color #: ", colorSensorLeftCache[0] & 0xFF);
        telemetry.addData("Color Sensor Right Color #: ", colorSensorRightCache[0] & 0xFF);
        telemetry.addData("Right Encoder: ", motorRight.getCurrentPosition());
        telemetry.addData("Left Encoder: ", motorLeft.getCurrentPosition());
        telemetry.update();

        //if(gamepad1.a == false) {
            //arcadeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x, .1);
        //} else {
            //arcadeDrive((gamepad1.left_stick_y / 2), (gamepad1.right_stick_x / 2), .1);
        //}
        driveStraightPID();
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        navx_device.close();
    }

    public void arcadeDrive(double y, double rotation, double deadZone){
        double leftPower;
        double rightPower;

        if((Math.abs(y) > deadZone) || (Math.abs(rotation) > deadZone)){
            if((Math.abs(y) > deadZone) && (Math.abs(rotation) < deadZone)){
                leftPower = y;
                rightPower = y;
            } else if((Math.abs(y) < deadZone) && (Math.abs(rotation) > deadZone)){
                leftPower = rotation;
                rightPower = -rotation;
            } else {
                leftPower = (y / 2) + (rotation / 2);
                rightPower = (y / 2) - (rotation / 2);
            }
        } else {
            leftPower = 0;
            rightPower = 0;
        }

        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);
    }

    public void driveStraightPID(){
        /* If possible, use encoders when driving, as it results in more */
        /* predictable drive system response.                           */
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        navx_device.zeroYaw();

        double error = 0 - navx_device.getYaw();

        if(error > 2.0 && error <= 5.0){
            motorRight.setPower(0.75);
            motorLeft.setPower(1.0);
        } else if(error > 5 && error <= 15){
            motorRight.setPower(0.5);
            motorLeft.setPower(1.0);
        } else if(error > 15 && error <= 90){
            motorRight.setPower(0.15);
            motorLeft.setPower(1.0);
        } else if(error < -2.0 && error >= -5.0){
            motorRight.setPower(0.75);
            motorLeft.setPower(1.0);
        } else if(error < -5.0 && error >= -15.0){
            motorRight.setPower(1.0);
            motorLeft.setPower(0.5);
        } else if(error < -15.0 && error >= -90){
            motorRight.setPower(1.0);
            motorLeft.setPower(0.15);
        } else if(error >= -2 && error < 2){
            motorRight.setPower(1);
            motorLeft.setPower(1);
        }

    }

    public double limit(double a) {
        return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
    }

}