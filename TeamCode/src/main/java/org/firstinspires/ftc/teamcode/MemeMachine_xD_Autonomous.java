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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

/**
 * This file illustrates the concept of driving up to a line and then stopping.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code shows using two different light sensors:
 *   The Primary sensor shown in this code is a legacy NXT Light sensor (called "light sensor")
 *   Alternative "commented out" code uses a MR Optical Distance Sensor (called "sensor_ods")
 *   instead of the LEGO sensor.  Chose to use one sensor or the other.
 *
 *   Setting the correct WHITE_THRESHOLD value is key to stopping correctly.
 *   This should be set half way between the light and dark values.
 *   These values can be read on the screen once the OpMode has been INIT, but before it is STARTED.
 *   Move the senso on asnd off the white line and not the min and max readings.
 *   Edit this code to make WHITE_THRESHOLD half way between the min and max.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: Auto Drive To Line", group="Pushbot")
public class MemeMachine_xD_Autonomous extends LinearVisionOpMode {

    /* Declare OpMode members. */
    Hardware robot = new Hardware();
    Lift lift = new Lift();
    Drive drive = new Drive();
    BallLoader ballLoader = new BallLoader();
    Shooter shooter = new Shooter();
    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    //    RangeSensor rangeSensor = new RangeSensor();

    //Frame counter
    int frameCount = 0;

    enum AutonomousState {idle, capBall, shootHigh, shootLow, pressBeacon, park};
    AutonomousState autonomousState;

//    static final double     WHITE_THRESHOLD = 0.2;  // spans between 0.1 - 0.5 from dark to light
//    static final double     APPROACH_SPEED  = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        lift.init(robot, gamepad1, telemetry);
        drive.init(robot, gamepad1, telemetry);
        shooter.init(robot, gamepad1, gamepad2, telemetry);
        ballLoader.init(robot, gamepad1, telemetry);
//        rangeSensor.init(robot);

        autonomousState = autonomousState.idle;

        telemetry.addData("Status", "Initialized");
        updateTelemetry(telemetry);

        //Wait for vision to initialize - this should be the first thing you do
        waitForVisionStart();

        /**
         * Set the camera used for detection
         * PRIMARY = Front-facing, larger camera
         * SECONDARY = Screen-facing, "selfie" camera :D
         **/
        this.setCamera(Cameras.PRIMARY);

        /**
         * Set the frame size
         * Larger = sometimes more accurate, but also much slower
         * After this method runs, it will set the "width" and "height" of the frame
         **/
        this.setFrameSize(new Size(900, 900));

        /**
         * Enable extensions. Use what you need.
         * If you turn on the BEACON extension, it's best to turn on ROTATION too.
         */
        enableExtension(VisionOpMode.Extensions.BEACON);         //Beacon detection
        enableExtension(VisionOpMode.Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(VisionOpMode.Extensions.CAMERA_CONTROL); //Manual camera control

        /**
         * Set the beacon analysis method
         * Try them all and see what works!
         */
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);

        /**
         * Set color tolerances
         * 0 is default, -1 is minimum and 1 is maximum tolerance
         */
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        /**
         * Set analysis boundary
         * You should comment this to use the entire screen and uncomment only if
         * you want faster analysis at the cost of not using the entire frame.
         * This is also particularly useful if you know approximately where the beacon is
         * as this will eliminate parts of the frame which may cause problems
         * This will not work on some methods, such as COMPLEX
         **/
        //beacon.setAnalysisBounds(new Rectangle(new Point(width / 2, height / 2), width - 200, 200));

        /**
         * Set the rotation parameters of the screen
         * If colors are being flipped or output appears consistently incorrect, try changing these.
         *
         * First, tell the extension whether you are using a secondary camera
         * (or in some devices, a front-facing camera that reverses some colors).
         *
         * It's a good idea to disable global auto rotate in Android settings. You can do this
         * by calling disableAutoRotate() or enableAutoRotate().
         *
         * It's also a good idea to force the phone into a specific orientation (or auto rotate) by
         * calling either setActivityOrientationAutoRotate() or setActivityOrientationFixed(). If
         * you don't, the camera reader may have problems reading the current orientation.
         */
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        /**
         * Set camera control extension preferences
         *
         * Enabling manual settings will improve analysis rate and may lead to better results under
         * tested conditions. If the environment changes, expect to change these values.
         */
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();

        //Wait for the match to begin
        waitForStart();

        while (opModeIsActive()) {//do actions in order of priority
            switch (autonomousState) {
                case idle:
                    //startup things

                    //next action
                    autonomousState = autonomousState.capBall;
                    break;

                case capBall:
                    //knock off cap ball
//                    if(beacon.getAnalysis.getColorString()) {

//                    }

                    //next action
                    if (elapsedTime.time() > 10000) { //needs value
                        autonomousState = autonomousState.pressBeacon;
                    }
                    break;

                case pressBeacon:
                    //press beacon

                    //next action
                    if (elapsedTime.time() > 15000) { //needs value
                        autonomousState = autonomousState.shootHigh;
                    }
                    break;

                case shootHigh:
                    //shoot for high goal

                    //next action
                    if (elapsedTime.time() > 20000) { //needs value
                        autonomousState = autonomousState.shootLow;
                    }
                    break;

                case shootLow:
                    //shoot for low goal

                    //next action
                    if (elapsedTime.time() > 25000) { //needs value
                        autonomousState = autonomousState.park;
                    }
                    break;

                case park:
                    //park

                    break;
            }
        }

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to our Light Sensor object.
//        lightSensor = hardwareMap.lightSensor.get("light sensor");                // Primary LEGO Light Sensor
        //  lightSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");  // Alternative MR ODS sensor.

        // turn on LED of light sensor.
//        lightSensor.enableLed(true);

        // Send telemetry message to signify robot waiting;
//        telemetry.addData("Status", "Ready to run");    //
//        telemetry.update();

//        // Wait for the game to start (driver presses PLAY)
//        while (!isStarted()) {
//
//            // Display the light level while we are waiting to start
//            telemetry.addData("Light Level", lightSensor.getLightDetected());
//            telemetry.update();
//            idle();
//        }

//        // Start the robot moving forward, and then begin looking for a white line.
//        robot.leftMotor.setPower(APPROACH_SPEED);
//        robot.rightMotor.setPower(APPROACH_SPEED);

        // run until the white line is seen OR the driver presses STOP;
//        while (opModeIsActive() && (lightSensor.getLightDetected() < WHITE_THRESHOLD)) {

            // Display the light level while we are looking for the line
//            telemetry.addData("Light Level",  lightSensor.getLightDetected());
//            telemetry.update();
//            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
//        }

        // Stop all motors
//        robot.leftMotor.setPower(0);
//        robot.rightMotor.setPower(0);
    }

    public void park() {

    }

    public void knockOffCapBall() {

    }

    public void shootHigh() {

    }

    public void shootLow() {

    }

    public void pressBeacon() {

    }
}
