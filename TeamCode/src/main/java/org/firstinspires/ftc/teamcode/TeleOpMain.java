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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
        motorRight, flywheelMotor, loadingMotor, intakeMotor;
    DeviceInterfaceModule deviceInterface1;
    DcMotorController motorController1, motorController2;
    OpticalDistanceSensor distanceSensor1;
    //TouchSensor touchSensor1;
    ColorSensor colorSensor1;
    ElapsedTime timer1 = new ElapsedTime();
    int rightError;
    int leftError;
    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable
    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read
    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;
    boolean isFlyWheelRunning;
    int numberOfCycles;
    int oldFlyWheelEnc;
    int newFlyWheelEnc;
    double flyWheelRate;
    boolean slowMode;
    boolean isIntakeRunning;

    @Override
    public void init() {
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight = hardwareMap.dcMotor.get("motorRight");
        //flywheelMotor = hardwareMap.dcMotor.get("flywheelMotor");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        //loadingMotor = hardwareMap.dcMotor.get("loadingMotor");           UnComment if we have a loading motor
        motorController1 = hardwareMap.dcMotorController.get("motorController1");
        motorController2 = hardwareMap.dcMotorController.get("motorController2");
        deviceInterface1 = hardwareMap.deviceInterfaceModule.get("interfaceModule1");
        //touchSensor1 = hardwareMap.touchSensor.get("touchSensor1");
        distanceSensor1 = hardwareMap.opticalDistanceSensor.get("distanceSensor1");
        colorSensor1 = hardwareMap.colorSensor.get("colorSensor1");
        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();
        colorSensor1.enableLed(false);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
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
        String colorLevels = Integer.toString(colorSensor1.red()) +","+ Integer.toString(colorSensor1.green())+"," + Integer.toString(colorSensor1.blue());
        telemetry.addData("Color Level: ", colorLevels);
        telemetry.addData("Distance Sensor: ", distanceSensor1.getLightDetected());
        telemetry.addData("Left Enc Reading: ", motorLeft.getCurrentPosition());
        telemetry.addData("Right Enc Reading: ", motorRight.getCurrentPosition());
        telemetry.addData("Is The FlyWheel Running?: ", isFlyWheelRunning);
        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);

        if(gamepad1.a ){
            runFlyWheel();
        } else {
            flywheelMotor.setPower(0);
            isFlyWheelRunning = false;
        }

        if(gamepad1.right_stick_button){
            slowMode = true;
        } else {
            slowMode = false;
        }

        if(gamepad1.left_trigger > .1){
            runIntake();
        } else if(gamepad1.right_trigger > .1) {
            outTake();
        } else {
            intakeMotor.setPower(0);
        }

        if(gamepad1.left_stick_y != 0 && gamepad1.right_stick_x != 0){
            arcadeDrive(-gamepad1.left_stick_y, -gamepad1.right_stick_x, 0.1);
        } else if(gamepad1.left_stick_y != 0 && gamepad1.right_stick_x != 0 && slowMode == true){
            slowMode(gamepad1.left_stick_y, gamepad1.right_stick_x, 0.1);
        } else {
            stopDriving();
        }

        telemetry.update();
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
    public void arcadeDrive(double y, double rotation, double deadZone){
        double leftPower;
        double rightPower;

        if((Math.abs(y) > deadZone) || (Math.abs(rotation) > deadZone)){
            if((Math.abs(y) > deadZone) && (Math.abs(rotation) < deadZone)){
                leftPower = y;
                rightPower = y;
            } else if((Math.abs(y) < deadZone) && (Math.abs(rotation) > deadZone)){
                leftPower = -rotation;
                rightPower = rotation;
            } else {
                leftPower = (y / 2) - (rotation / 2);
                rightPower = (y / 2) + (rotation / 2);
            }
        } else {
            leftPower = 0;
            rightPower = 0;
        }

        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);
    }
    public void stopDriving(){
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }
    public void setPower(double power){
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }
    public void bangBangDrive(int target) {
        rightError = target - motorRight.getCurrentPosition();
        leftError = target - motorLeft.getCurrentPosition();
        //leftError = -leftError;
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
        }
        else if(leftError > -250 && leftError <= -175){
            motorLeft.setPower(-.13);
        }
        else if(leftError > -175 && leftError <= -150){
            motorLeft.setPower(-.09);
        }
        else if(leftError > -150 && leftError <= -50){
            motorLeft.setPower(0);
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

        }
        else if(leftError < 250 && leftError >= 175){
            motorLeft.setPower(.13);

        }
        else if(leftError < 175 && leftError >= 150){
            motorLeft.setPower(.09);
        }
        else if(leftError < 150 && leftError >= 50){
            motorLeft.setPower(0);
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
        }
        else if(rightError > -250 && rightError <= -175){
            motorRight.setPower(-.13);
        }
        else if(rightError > -175 && rightError <= -150){
            motorRight.setPower(-.09);
        }
        else if(rightError > -150 && rightError <= -50){
            motorRight.setPower(0);
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

        }
        else if(rightError < 250 && rightError >= 175){
            motorRight.setPower(.13);

        }
        else if(rightError < 175 && rightError >= 150){
            motorRight.setPower(.09);
        }
        else if(rightError < 150 && rightError >= 50){
            motorRight.setPower(0);
        }
    }
    public void runFlyWheel(){
        flywheelMotor.setPower(1);
        isFlyWheelRunning = true;
    }
    public double getFlyWheelRate(){
        if(numberOfCycles == 1){
            oldFlyWheelEnc = flywheelMotor.getCurrentPosition();
        } else if(numberOfCycles == 500){
            newFlyWheelEnc = flywheelMotor.getCurrentPosition();
            flyWheelRate = newFlyWheelEnc - oldFlyWheelEnc;
            numberOfCycles = 0;
        }
        return flyWheelRate;

    }
    public boolean isFlyWheelAtSpeed(){
        if(getFlyWheelRate() > 20 && getFlyWheelRate() < 25){
            return true;
        }
        return false;
    }
    public void loadBall(){
        if(isFlyWheelAtSpeed() && gamepad1.a){
            timer1.reset();
            timer1.startTime();
            if(timer1.seconds() > .5 && timer1.seconds() <= 1){
                loadingMotor.setPower(.5);
            } else {
                loadingMotor.setPower(0);
            }
        }
    }
    public void runIntake(){
        intakeMotor.setPower(gamepad1.left_trigger);
        isIntakeRunning = true;
    }
    public void outTake(){
        intakeMotor.setPower(gamepad1.right_trigger);
    }
    public void slowMode(double y, double rotation, double deadZone){
        double leftPower;
        double rightPower;

        if((Math.abs(y) > deadZone) || (Math.abs(rotation) > deadZone)){
            if((Math.abs(y) > deadZone) && (Math.abs(rotation) < deadZone)){
                leftPower = y / 3;
                rightPower = y / 3;
            } else if((Math.abs(y) < deadZone) && (Math.abs(rotation) > deadZone)){
                leftPower = -rotation / 3;
                rightPower = rotation / 3;
            } else {
                leftPower = (y / 2) - (rotation / 2);
                rightPower = (y / 2) + (rotation / 2);
            }
        } else {
            leftPower = 0;
            rightPower = 0;
        }

        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);
    }
}
