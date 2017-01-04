package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.CameraCalibration;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.Arrays;

/**
 * Created by swat on 12/9/2016.
 */

@TeleOp(name="Vision Code", group="Iterative Opmode")
public class VuforiaOp extends LinearOpMode {
    public final static Scalar blueLow = new Scalar(108,0,220);
    public final static Scalar blueHigh = new Scalar(178,255,255);

    public int BEACON_NOT_VISIBLE = 0;
    public int BEACON_RED_BLUE = 1;
    public int BEACON_BLUE_RED = 2;
    public int BEACON_ALL_BLUE = 3;
    public int BEACON_NO_BLUE = 4;


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motorLeft = hardwareMap.dcMotor.get("motorLeft");
        DcMotor motorRight = hardwareMap.dcMotor.get("motorRight");
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(motorRight.getCurrentPosition() != 0 || motorLeft.getCurrentPosition() != 0){

        }
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AWVhcNf/////AAAAGSp6OASlY0W4vadoxwWt+YlkbrXsJrpinFzVs2T5iuTkkXN/ju89SJBN1nUrdCU0S3Dl+X9xmXn/QFTzL4Oz3vJ/DXtmZICEC8TdxhHOr4gL/1L8DS3AI1WIXKVwN5KQDvKSKr6IACT+v8i/QdbfPeziUgmWqJjRPSPdngrGtxyD4gROSdd3l+/4Wc5qlTGkMDP4EbgoxTgThM9Ff7/OJJ6qkuPEg5k4NbTFU+AtosrKsI1JqLLBjdNQlGr3qpFuHQenDiybTd6fyxz4OPXwvBimXJ/qpQE7FUzU+cHy7Ac4R0QfF+Yfhy4ND1092WmO3e+PPYWfrK+otqcusrGO1VWTWwn/wgIjQUbbaFNni/U4";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizerImplSubclass vuforia = new VuforiaLocalizerImplSubclass(params);

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");

        VuforiaTrackableDefaultListener wheels = (VuforiaTrackableDefaultListener) beacons.get(0).getListener();

        beacons.activate();

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeft.setPower(0.2);
        motorRight.setPower(0.2);

        while(opModeIsActive() && wheels.getRawPose() == null){
            idle();
        }
        motorLeft.setPower(0.0);
        motorRight.setPower(0.0);

        //Analyze the beacon!

        int config = getBeaconConfig(getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), wheels, vuforia.getCameraCalibration());

        VectorF angles = anglesFromTarget(wheels);

        if(config == BEACON_BLUE_RED){
            //drive to left side
        } else {
            //drive to right side
        }
        //50 cm from wall and 3rd is side to side.
        VectorF trans = navOffWall(wheels.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500,0,0));

        if(trans.get(0) > 0){
            motorLeft.setPower(0.2);
            motorRight.setPower(-0.2);
        } else {
            motorLeft.setPower(-0.2);
            motorRight.setPower(0.2);
        }

        do{
            if(wheels.getPose() != null){
                trans = navOffWall(wheels.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500,0,0));
            }
            idle();
        } while (opModeIsActive() && Math.abs(trans.get(0)) > 30);

        motorLeft.setPower(0);
        motorRight.setPower(0);

        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Center of turn in is 15 cm off center of phone. WIll change.
        motorLeft.setTargetPosition((int) (motorLeft.getCurrentPosition() + ((Math.hypot(trans.get(0), trans.get(2)) + 127) / 239.389 * 280)));
        motorRight.setTargetPosition((int) (motorRight.getCurrentPosition() + ((Math.hypot(trans.get(0), trans.get(2)) + 127) / 239.389 * 280)));

        motorLeft.setPower(0.3);
        motorRight.setPower(0.3);

        while(opModeIsActive() && motorLeft.isBusy() && motorRight.isBusy()){
            idle();
        }

        motorLeft.setPower(0.0);
        motorRight.setPower(0.0);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive() && (wheels.getPose() == null || Math.abs(wheels.getPose().getTranslation().get(0)) > 10)){
            if(wheels.getPose() != null){
                if(wheels.getPose().getTranslation().get(0) > 0){
                    motorLeft.setPower(-0.2);
                    motorRight.setPower(0.2);
                } else {
                    motorLeft.setPower(0.2);
                    motorRight.setPower(-0.2);
                }
            } else {
                motorLeft.setPower(-0.2);
                motorRight.setPower(0.2);
            }
        }

        motorLeft.setPower(0);
        motorRight.setPower(0);

        while(opModeIsActive()){

            if(vuforia.rgb != null){
                Bitmap bm = Bitmap.createBitmap(vuforia.rgb.getWidth(), vuforia.rgb.getHeight(), Bitmap.Config.RGB_565);
                bm.copyPixelsFromBuffer(vuforia.rgb.getPixels());


            }

            for(VuforiaTrackable beac : beacons){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

                if(pose != null){
                    VectorF translation = pose.getTranslation();

                    telemetry.addData(beac.getName() + "-Translation", translation);
                    //if in landscape 0,2
                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));

                    telemetry.addData(beac.getName() + "Degrees", degreesToTurn);
                    Matrix34F rawPose = new Matrix34F();

                    float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0,12);
                    rawPose.setData(poseData);

                    Vec2F upperLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-127,92,0));
                    Vec2F upperRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(127,92,0));
                    Vec2F lowerRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(127,-92,0));
                    Vec2F lowerLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-127,-92,0));

                    telemetry.addData("Current State: ", config);
                }

            }
            telemetry.update();
        }
    }

    public Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int pixelFormat){

        long numImgs = frame.getNumImages();
        for(int i = 0;i< numImgs;i++){
            if(frame.getImage(i).getFormat() == pixelFormat){
                return frame.getImage(i);
            }
        }

        return null;
    }

    public VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall){
        return new VectorF((float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))), trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
    }

    public VectorF anglesFromTarget(VuforiaTrackableDefaultListener image){
        float [] data = image.getRawPose().getData();
        float [] [] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};
        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
        return new VectorF((float)thetaX, (float)thetaY, (float)thetaZ);
    }

    public int getBeaconConfig(Image img, VuforiaTrackableDefaultListener beacon, CameraCalibration camCal){
        OpenGLMatrix pose = beacon.getRawPose();
        if(pose != null && img != null && img.getPixels() != null){
            Matrix34F rawPose = new Matrix34F();
            float [] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
            rawPose.setData(poseData);

            float [] [] corners = new float[4] [2];

            corners[0] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, 276, 0)).getData(); //upper left
            corners[1] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, 276, 0)).getData(); //upper right
            corners[2] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, 92, 0)).getData(); //bottom right
            corners[3] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, 92, 0)).getData(); //bottom left

            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(img.getPixels());

            Mat crop = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3);
            Utils.bitmapToMat(bm,crop);

            float x= Math.min(Math.min(corners[1][0], corners[3][0]), Math.min(corners[0][0], corners[2][0]));
            float y= Math.min(Math.min(corners[1][1], corners[3][1]), Math.min(corners[0][1], corners[2][1]));
            float width= Math.max(Math.abs(corners[0][0] - corners[2][0]), Math.abs(corners[1][0]- corners[3][0]));
            float height= Math.max(Math.abs(corners[0][1] - corners[2][1]), Math.abs(corners[1][1] - corners[3][1]));

            x = Math.max(x,0);
            y= Math.max(y,0);

            width = (x = width > crop.cols()? crop.cols() - x: width);
            height = (y+ height > crop.rows())? crop.rows() - y : height;

            Mat cropped = new Mat(crop, new Rect((int) x, (int)y, (int) width, (int) height));

            Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_RGB2HSV_FULL);

            Mat mask = new Mat();
            Core.inRange(cropped, blueLow, blueHigh, mask);
            Moments mmnts = Imgproc.moments(mask, true);

            if(mmnts.get_m00() > mask.total() * 0.8){
                return BEACON_ALL_BLUE;
            } else if(mmnts.get_m00() < mask.total() * 0.1){
                return BEACON_NO_BLUE;
            }

            if((mmnts.get_m01() / mmnts.get_m00()) < cropped.rows() / 2){
                return BEACON_RED_BLUE;
            } else {
                return BEACON_BLUE_RED;
            }



        }

        return BEACON_NOT_VISIBLE;
    }
}
