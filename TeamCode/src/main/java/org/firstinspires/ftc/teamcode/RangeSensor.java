/**
 * Created by JJ hehe XD
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.R.attr.left;
import static android.R.attr.right;

@TeleOp(name = "MemeMachine_xD_Range_Sensor", group = "Indeterminate")
public class RangeSensor {

    Hardware robot = new Hardware();

    public void init(Hardware ahwMap) {
        robot = ahwMap;

//        telemetry.addData("Status", "Initialized");
//        updateTelemetry(telemetry);
    }

    public void loop() {

    }
}