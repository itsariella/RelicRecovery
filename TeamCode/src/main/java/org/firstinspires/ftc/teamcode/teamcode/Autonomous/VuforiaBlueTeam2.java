/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.teamcode.Libraries.PushbotHardware;


/**
 * This file illustrates the concept of driving a path based on GyroBlueTeam1 heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the GyroBlueTeam1 correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the GyroBlueTeam1 Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the GyroBlueTeam1.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
//hi
@Autonomous(name="Vuforia Blue 2", group="Pushbot")
public class VuforiaBlueTeam2 extends LinearOpMode {

    /* Declare OpMode members. */
    ColorSensor colorSensor;
    PushbotHardware robot   = new PushbotHardware();   // Use a Pushbot's hardware
    BNO055IMU imu;                   // Additional GyroBlueTeam1 device
    VuforiaLocalizer vuforia;

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.937 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.3 ;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.
    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable


    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);

        //vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vparameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vparameters.vuforiaLicenseKey = "AcVwYcb/////AAAAGWEdxQf3YU2/lW5yeYD13aeDV1xztWzGXEMrenB3Ax0/LWpgWHKVby7cXbxJVFwwxDnktV0N/HDZ9gXDhz0reHdJ++0LRfTmDRWcbPcCRCZJk5FV3WvLmkUz8zyGUTDQabO3ooR2oaDD/HqZDLHXunzpIGWjJKOYSNlUSRE0Xy2LwTIqOZEprAOhZnuMolNTuTZa3Z5Ql7C2MmJqkpfvrBEIUS/+D0Ozt9LHD20zJKH45TMqa5EkJl2mqf3me+lFhhgJa0ff/gPBomVoayhQUvp72E6nOwF/nVLBIAf/GiZ2DOdW7DGCqrl5cBlvL7UsFkqY2526WTbDtahssM+7ABgUKlDyBOgd0/15JNLv2/CL";
        vparameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(vparameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating GyroBlueTeam1");    //
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parameters.mode = BNO055IMU.SensorMode.IMU;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        colorSensor.enableLed(true);
        imu.initialize(parameters);


        // make sure the gyro is calibrated before continuing
        /*while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        } */
        relicTrackables.activate(); //activate before start button is pressed

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();


        waitForStart();

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        // Wait for the game to start (Display GyroBlueTeam1 value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", imu.getAngularOrientation());
            telemetry.update();
        }

        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn

        while (opModeIsActive()) {

            openArms();
            moveOutCatchers();
            sleep(1500);
            armDown();
            sleep(750);
            knockJewel();
            sleep(750);
            armUp();

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                if(vuMark == RelicRecoveryVuMark.CENTER){
                    telemetry.addLine("Going Center");

                    gyroDrive(DRIVE_SPEED,-24,0); //drive towards box
                    gyroTurn(TURN_SPEED,90); //turn left
                    gyroHold(TURN_SPEED,90,.5);
                    gyroDrive(DRIVE_SPEED,12,90);//drive towards center of the box
                    gyroTurn(TURN_SPEED,180);
                    gyroHold(TURN_SPEED,180,.5);
                    gyroDrive(DRIVE_SPEED,8,180); //drive towards box
                    scoreGlyph(); //score
                    sleep(1000);
                    stopIntake();
                    gyroDrive(DRIVE_SPEED,-7,180);//drive away from box
                    gyroDrive(DRIVE_SPEED,9,180); //push glyph back in
                    gyroDrive(DRIVE_SPEED,-5,180); //back out

                    telemetry.addData("Path Center", "Complete");
                    telemetry.update();

                }
                else if(vuMark == RelicRecoveryVuMark.LEFT) {
                    telemetry.addLine("Going Left");

                    gyroDrive(DRIVE_SPEED,-24,0); //drive towards box
                    gyroTurn(TURN_SPEED,90); //turn right
                    gyroHold(TURN_SPEED,90,.5);
                    gyroDrive(DRIVE_SPEED,5,90);//drive towards left
                    gyroTurn(TURN_SPEED,180);
                    gyroHold(TURN_SPEED,180,.5);
                    gyroDrive(DRIVE_SPEED,8,180); //drive towards box
                    scoreGlyph();
                    sleep(1000);
                    stopIntake();
                    gyroDrive(DRIVE_SPEED,-7,180);//drive away from box
                    gyroDrive(DRIVE_SPEED,9,180); //push glyph back in
                    gyroDrive(DRIVE_SPEED,-5,180); //back out

                    telemetry.addData("Path Left", "Complete");
                    telemetry.update();

                }
                else if(vuMark == RelicRecoveryVuMark.RIGHT) {
                    telemetry.addLine("Going Right");

                    gyroDrive(DRIVE_SPEED,-24,0); //drive towards box
                    gyroTurn(TURN_SPEED,90); //turn right
                    gyroHold(TURN_SPEED,90,.5);
                    gyroDrive(DRIVE_SPEED,19,90); //drive towards left
                    gyroTurn(TURN_SPEED,180);
                    gyroHold(TURN_SPEED,180,.5);
                    gyroDrive(DRIVE_SPEED,9,180); //drive towards forward
                    scoreGlyph();
                    sleep(1000);
                    stopIntake();
                    gyroDrive(DRIVE_SPEED,-7,180);//drive away from box
                    gyroDrive(DRIVE_SPEED,9,180); //push glyph back in
                    gyroDrive(DRIVE_SPEED,-5,180); //back out

                    telemetry.addData("Path Right", "Complete");
                    telemetry.update();

                }
            }
            else {
                telemetry.addData("VuMark", "not visible");

                gyroDrive(DRIVE_SPEED,-24,0); //drive towards box
                gyroTurn(TURN_SPEED,90); //turn left
                gyroHold(TURN_SPEED,90,.5);
                gyroDrive(DRIVE_SPEED,12,90); //drive towards the center
                gyroTurn(TURN_SPEED,180); //face cryptobox
                gyroHold(TURN_SPEED,180,.5);
                gyroDrive(DRIVE_SPEED,8,180); //drive forward
                scoreGlyph(); //score
                sleep(1000);
                stopIntake();
                gyroDrive(DRIVE_SPEED,-7,180);//drive away from box
                gyroDrive(DRIVE_SPEED,9,180); //push glyph back in
                gyroDrive(DRIVE_SPEED,-5,180); //back out

                telemetry.addData("Path", "Complete");
                telemetry.update();

            }
            telemetry.update();
            sleep(15000);
        }
    }


   /**
    *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
    * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from current heading.
    */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     backLeftTarget;
        int     frontLeftTarget;
        int     frontRightTarget;
        int     backRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            backLeftTarget = robot.backLeft.getCurrentPosition() + moveCounts;
            frontLeftTarget = robot.frontLeft.getCurrentPosition() + moveCounts;
            backRightTarget = robot.backRight.getCurrentPosition() + moveCounts;
            frontRightTarget = robot.frontRight.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.frontLeft.setTargetPosition(frontLeftTarget);
            robot.backLeft.setTargetPosition(backLeftTarget);
            robot.frontRight.setTargetPosition(frontRightTarget);
            robot.backRight.setTargetPosition(backRightTarget);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.frontLeft.setPower(speed);
            robot.frontRight.setPower(speed);
            robot.backLeft.setPower(speed);
            robot.backRight.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                   (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backRight.isBusy() && robot.backLeft.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.frontLeft.setPower(leftSpeed);
                robot.backLeft.setPower(leftSpeed);
                robot.frontRight.setPower(rightSpeed);
                robot.backRight.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      frontLeftTarget,  backLeftTarget, backLeftTarget, backRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.frontLeft.getCurrentPosition(),
                                                             robot.frontRight.getCurrentPosition(), robot.backLeft.getCurrentPosition(), robot.backRight.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeft.setPower(0);
            robot.backLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
    }

    public void knockJewel() {

        if (colorSensor.blue() > colorSensor.red()) {
            gyroTurn(TURN_SPEED, 10);
            gyroHold(TURN_SPEED, 10, 0.5);
            armUp();
            gyroTurn(TURN_SPEED, 0);
            gyroHold(TURN_SPEED, 0, 1);
        } else {
            gyroTurn(TURN_SPEED, -10);
            gyroHold(TURN_SPEED, -10, 0.5);
            armUp();
            gyroTurn(TURN_SPEED, 0);
            gyroHold(TURN_SPEED, 0, 1);
        }
    }

    public void armDown(){
        robot.jewelArm2.setPosition(0);
    }
    public void armUp(){
        robot.jewelArm2.setPosition(1);
    }
    public void grab() {
        robot.firstStage1.setPosition(0);
        robot.firstStage2.setPosition(0);
    }
    public void release(){
        robot.firstStage1.setPosition(0.4);
        robot.firstStage2.setPosition(0.4);
    }
    public void setUp(){
        robot.intakeLeft.setPower(1);
        robot.intakeRight.setPower(1);
    }
    public void stopIntake(){
        robot.intakeRight.setPower(0);
        robot.intakeLeft.setPower(0);
    }
    public void liftUp(){
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        robot.lift.setTargetPosition(1500);

        //set mode
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //set power
        robot.lift.setPower(1);

        while(opModeIsActive() && robot.lift.isBusy()){
            telemetry.addData("Path2",  "Running at %7d", robot.lift.getCurrentPosition());
            telemetry.update();

            idle();
        }
        robot.lift.setPower(0);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void openArms(){
        robot.firstStage1.setPosition(.5); //glyph arms open
        robot.firstStage2.setPosition(.5);
    }

    public void moveOutCatchers() {
        robot.catcherLeft.setPosition(1);
        robot.catcherRight.setPosition(1);
    }
    public void scoreGlyph () {
        robot.intakeRight.setPower(1);
        robot.intakeLeft.setPower(1);
        sleep(1000);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.frontLeft.setPower(leftSpeed);
        robot.backLeft.setPower(leftSpeed);
        robot.backRight.setPower(rightSpeed);
        robot.frontRight.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last GyroBlueTeam1 Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - imu.getAngularOrientation().firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}
