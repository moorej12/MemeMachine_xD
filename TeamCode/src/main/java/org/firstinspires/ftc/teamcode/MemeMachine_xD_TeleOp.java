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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.R.attr.left;
import static android.R.attr.right;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "MemeMachine_xD_TeleOp", group = "Indeterminate")
public class MemeMachine_xD_TeleOp extends OpMode {

    Hardware robot = new Hardware();
//    Lift lift = new Lift();
    Drive drive = new Drive();
    BallLoader ballLoader = new BallLoader();
    Shooter shooter = new Shooter();
//    RangeSensor rangeSensor = new RangeSensor();

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        robot.init(hardwareMap);
//        lift.init(robot, gamepad1, telemetry);
        drive.init(robot, gamepad1, telemetry);
        shooter.init(robot, gamepad1, gamepad2, telemetry);
//        rangeSensor.init(robot);
        ballLoader.init(robot, gamepad1, telemetry);

        telemetry.addData("Status", "Initialized");
        updateTelemetry(telemetry);
    }

    /*
       * Code to run when the op mode is first enabled goes here
       * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
       */
    @Override
    public void init_loop() {

    }

    /*
     * This method will be called ONCE when start is pressed
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        double left;
        double right;

//        lift.loop();
        ballLoader.loop();
        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;
//        robot.leftRearMotor.setPower(left);
//        robot.rightRearMotor.setPower(right);
        robot.leftFrontMotor.setPower(left);
        robot.rightFrontMotor.setPower(right);

        drive.loop();
        shooter.loop();

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        updateTelemetry(telemetry);
    }
}
