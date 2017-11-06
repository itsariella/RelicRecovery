package org.firstinspires.ftc.teamcode.teamcode.Libraries;
import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.HINT;
import com.vuforia.Matrix34F;
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
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.teamcode.FTCVuforiaDemo.VuforiaLocalizerImplSubclass;

import java.util.Arrays;


/**
 * Created by Ariella on 6/5/2017.
 */

@Autonomous(name="VuforiaOp", group="Pushbot")
public class VuforiaOp extends AutoEncoderHybrid {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); //displays camera view
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; // front camera
        params.vuforiaLicenseKey = "AcVwYcb/////AAAAGWEdxQf3YU2/lW5yeYD13aeDV1xztWzGXEMrenB3Ax0/LWpgWHKVby7cXbxJVFwwxDnktV0N/HDZ9gXDhz0reHdJ++0LRfTmDRWcbPcCRCZJk5FV3WvLmkUz8zyGUTDQabO3ooR2oaDD/HqZDLHXunzpIGWjJKOYSNlUSRE0Xy2LwTIqOZEprAOhZnuMolNTuTZa3Z5Ql7C2MmJqkpfvrBEIUS/+D0Ozt9LHD20zJKH45TMqa5EkJl2mqf3me+lFhhgJa0ff/gPBomVoayhQUvp72E6nOwF/nVLBIAf/GiZ2DOdW7DGCqrl5cBlvL7UsFkqY2526WTbDtahssM+7ABgUKlDyBOgd0/15JNLv2/CL";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizerImplSubclass vuforia = new VuforiaLocalizerImplSubclass(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4); //can possibly see 4 targets, dont throw error

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17"); // FTCRobotController > src > assets > xml file
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Gears");
        beacons.get(3).setName("Legos");

        VuforiaTrackableDefaultListener tools = (VuforiaTrackableDefaultListener) beacons.get(1).getListener();

        waitForStart();

        beacons.activate();

        //DriveForward(.3, 2240);

        while (opModeIsActive() && tools.getRawPose() == null) {
            idle();
        }

        StopDriving();


        VectorF angles = anglesFromTarget(tools);
        VectorF trans = navOffWall(tools.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500, 0, 0));


        if (trans.get(0) > 0) {
            TurnRight(.2);
        } else {
            TurnLeft(.2);
        }

        do {
            if (tools.getPose() != null) {
                trans = navOffWall(tools.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500, 0, 0)); //Vector where you want to be away from the wall (50 cm away, 0 side to side change
            }
            idle();
        } while (opModeIsActive() && Math.abs(trans.get(0)) > 30); //get x value

        StopDriving();

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setTargetPosition((int)(frontLeft.getCurrentPosition() * ((Math.hypot(trans.get(0), trans.get(2)) + 150) / 100 * 1120))); //15 cm off the center of the robot, /100 wheel circumference *560 ticks per rotation
        frontRight.setTargetPosition((int)(frontLeft.getCurrentPosition() * ((Math.hypot(trans.get(0), trans.get(2)) + 150) / 100 * 1120)));
        backLeft.setTargetPosition((int)(frontLeft.getCurrentPosition() * ((Math.hypot(trans.get(0), trans.get(2)) + 150) / 100 * 1120)));
        backRight.setTargetPosition((int)(frontLeft.getCurrentPosition() * ((Math.hypot(trans.get(0), trans.get(2)) + 150) / 100 * 1120)));

        //DriveForwards(0.3);

        while (opModeIsActive() && frontLeft.isBusy()) {
            idle();
        }

        StopDriving();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //turn again till face wall below
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*

        while (opModeIsActive() && wheels.getPose() == null || Math.abs(wheels.getPose().getTranslation().get(0)) > 10) {
            if (wheels != null) {
                if (wheels.getPose().getTranslation().get(0) > 0) {
                    TurnRight(.3);
                } else {
                    TurnLeft(.3);
                }
            }
            else {
                TurnRight(.3);
            }
        }

        */

        while (opModeIsActive()) {

            if (vuforia.rgb != null) {
                Bitmap bm = Bitmap.createBitmap(vuforia.rgb.getWidth(), vuforia.rgb.getHeight(), Bitmap.Config.RGB_565);
                bm.copyPixelsFromBuffer(vuforia.rgb.getPixels());
            }
            for (VuforiaTrackable beac : beacons) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getRawPose();

                if (pose != null) {
                    Matrix34F rawPose = new Matrix34F();
                    float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
                    rawPose.setData(poseData);

                    Vec2F upperLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-127, 92, 0));
                    Vec2F upperRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(127, 92, 0));
                    Vec2F lowerLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-127, -92, 0));
                    Vec2F lowerRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-127, 92, 0));

                    /*VectorF translation = pose.getTranslation();

                    telemetry.addData(beac.getName() + "-Translation", translation);

                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));

                    telemetry.addData(beac.getName() + "-Degrees", degreesToTurn); */


                }
            }
            telemetry.update();
        }
    }

    public VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall) {
        return new VectorF((float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))), trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
    }

    public VectorF anglesFromTarget(VuforiaTrackableDefaultListener image) {
        float[] data = image.getRawPose().getData();
        float[][] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};
        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
        return new VectorF((float) thetaX, (float) thetaY, (float) thetaZ);
    }
}



