package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.Sensors;
import org.firstinspires.ftc.teamcode.utils.move;

@Autonomous(name="A C 0", group="Autonomous Finals")
//@Disabled
public class driveForward extends LinearOpMode{



    public void runOpMode()
    {


        move drive = new move(this);
        /*
        drive.turnRight(5, 0.15);
        drive.lowerRaisin();
        sleep(1000);
        drive.raisein();
        drive.turnRight(-5, 0.15);
        drive.turnRight(-30,0.15);
        Sensors.vuMark();
        drive.turnRight(20,0.15);
        */
        drive.right(-25, -0.5);
        drive.forward(20,0.15);
        drive.right(10, 0.5);
        drive.toColumn();
        drive.release();
        drive.forward(-6,-0.3);

    }
}
