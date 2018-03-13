package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
/**
 * Created by Patrick on 3/12/2018.
 */
@TeleOp (name = "liuTeleOp")
public class liuTeleOp extends OpMode{
    // defines the motor objects
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor lifter;
    // variables for motor power
    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;
    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;
    @Override
    public void init() {
        //initializes motors in configuration
        backLeft = hardwareMap.dcMotor.get("back left motor");
        backRight = hardwareMap.dcMotor.get("back right motor");
        frontLeft = hardwareMap.dcMotor.get("front left motor");
        frontRight = hardwareMap.dcMotor.get("front right motor");
        lifter = hardwareMap.dcMotor.get("lift motor");
        
        // puts the left motors in reverse, so it allows for turning
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void loop(){

    }
}
