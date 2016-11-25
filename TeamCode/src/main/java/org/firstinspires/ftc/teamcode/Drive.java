
/**
 * Created by James on 2016-10-18.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.R.attr.left;
import static android.R.attr.right;

@TeleOp(name = "MemeMachine_xD_Drive", group = "Indeterminate")
public class Drive {

    Hardware robot = new Hardware();

    public void init(Hardware ahwMap) {
        robot = ahwMap;

//        telemetry.addData("Status", "Initialized");
//        updateTelemetry(telemetry);
    }

    public void loop() {

        double left;
        double right;

//        left = gamepad1.left_stick_y;
//        right = gamepad1.right_stick_y;
//        robot.leftRearMotor.setPower(left);
//        robot.rightRearMotor.setPower(right);
//        robot.leftFrontMotor.setPower(left);
//        robot.rightFrontMotor.setPower(right);

//        telemetry.addData("left front",  "%.2f", left);
//        telemetry.addData("right front", "%.2f", right);
//        telemetry.addData("left rear",  "%.2f", left);
//        telemetry.addData("right rear", "%.2f", right);

    }
}
