package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.move;
@Autonomous(name = "Moko test")
public class driveStraight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        move drive = new move(this);
        drive.right(120,0.6);
    }



}
