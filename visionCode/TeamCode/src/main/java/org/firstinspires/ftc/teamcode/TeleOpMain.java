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
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
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
        motorRight, loadingMotor, flyWheelMotorFront, flyWheelMotorBack, intakeMotor;
    DeviceInterfaceModule deviceInterface1;
    DcMotorController motorController1, motorController2;
    ColorSensor colorSensor1;
    ElapsedTime timer1 = new ElapsedTime();
    int rightError;
    int leftError;
    boolean isFlyWheelRunning;
    int numberOfCycles;
    int oldFlyWheelEnc;
    int newFlyWheelEnc;
    double flyWheelRate;
    boolean isIntakeRunning;
    private I2cDevice colorSensorLeft;
    private I2cDeviceSynch colorSensorLeftReader;
    private I2cDevice colorSensorRight;
    private I2cDeviceSynch colorSensorRightReader;
    private byte[] colorSensorLeftCache;
    private byte[] colorSensorRightCache;

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
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorController1 = hardwareMap.dcMotorController.get("motorController1");
        motorController2 = hardwareMap.dcMotorController.get("motorController2");
        deviceInterface1 = hardwareMap.deviceInterfaceModule.get("interfaceModule1");
        colorSensor1 = hardwareMap.colorSensor.get("colorSensor1");
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
        colorSensorRightCache = colorSensorRightReader.read(0x1c, 1);
        colorSensorLeftCache = colorSensorLeftReader.read(0x1c, 1);
        telemetry.addData("Color Sensor Left Color #: ", colorSensorLeftCache[0] & 0xFF);
        telemetry.addData("Color Sensor Right Color #: ", colorSensorRightCache[0] & 0xFF);

        //String colorLevels = Integer.toString(colorSensor1.red()) +","+ Integer.toString(colorSensor1.green())+"," + Integer.toString(colorSensor1.blue());
        //telemetry.addData("Color Level: ", colorLevels);
        telemetry.addData("Left Enc Reading: ", motorLeft.getCurrentPosition());
        telemetry.addData("Right Enc Reading: ", motorRight.getCurrentPosition());
        telemetry.addData("Is The FlyWheel Running?: ", isFlyWheelRunning);
        telemetry.addData("Color R: ", colorSensor1.red());
        telemetry.addData("Color G: ", colorSensor1.green());
        telemetry.addData("Color B: ", colorSensor1.blue());
        telemetry.addData("Color Alpha: ", colorSensor1.alpha());
        telemetry.update();
        colorSensor1.enableLed(true);
        arcadeDrive(-gamepad1.left_stick_y, -gamepad1.right_stick_x, .1);
        if(gamepad1.a){
            bangBangDrive(12);
        }
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

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

    public void runShootingSpeed(){
        flyWheelMotorBack.setPower(.8);
        flyWheelMotorFront.setPower(.8);
        isFlyWheelRunning = true;
    }
    public void runIdleSpeed(){
        flyWheelMotorBack.setPower(.3);
        flyWheelMotorFront.setPower(.3);
        isFlyWheelRunning = false;
    }
    public double getFlyWheelRate(){
        if(numberOfCycles == 1){
            oldFlyWheelEnc = flyWheelMotorBack.getCurrentPosition();
        } else if(numberOfCycles == 500){
            newFlyWheelEnc = flyWheelMotorBack.getCurrentPosition();
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

            } else {
                loadingMotor.setPower(0);
            }
        }
    }

    public void intake(){
        intakeMotor.setPower(Math.abs(gamepad1.right_trigger));
        isIntakeRunning = true;
    }
    public void outTake(){
        intakeMotor.setPower(gamepad1.left_trigger);

    }
}
