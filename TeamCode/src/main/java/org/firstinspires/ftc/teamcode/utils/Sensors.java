package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.configuration.ExpansionHubMotorControllerVelocityParams;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.utils.oldSensors.gyro;


public class Sensors {

    public static double gyrochange;

    private static ElapsedTime runtime = new ElapsedTime();
    public static double gyroInitial;
    public static BNO055IMU gyro;
    static boolean red;

    public static double driverOffset = 0;
    public static Orientation angles;
    private static LinearOpMode opMode;
    private static Telemetry telemetry;


    /**
     *
     * @param _opMode to get FTC data
     * @param reds whether or not we are on the red team
     */
    public static void initialize(OpMode _opMode, boolean reds) {
        if (_opMode instanceof LinearOpMode) {
            opMode = (LinearOpMode) _opMode;
        } else {
            return;
        }
        HardwareMap hardware_map = opMode.hardwareMap;
        red = reds;

        telemetry = opMode.telemetry;

        gyroThread gThread = new gyroThread();
        gThread.start();

        BNO055IMU.Parameters IMUparams = new BNO055IMU.Parameters();
        IMUparams.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        IMUparams.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUparams.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        IMUparams.loggingEnabled      = true;
        IMUparams.loggingTag          = "IMU";
        IMUparams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = hardware_map.get(BNO055IMU.class, "imu");

        driverOffset = 0;

        runtime.reset();

        gyro.initialize(IMUparams);
        sleep(100);
        angles   = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        telemetry.addData("angle", angles);
        gyroInitial = angles.thirdAngle;
        ((LinearOpMode) _opMode).waitForStart();
        gyroDriftRead();
    }




    public static void gyroDriftRead() {

        if(angles.thirdAngle == 0) {
            gyrochange = 0;
        } else if (angles.thirdAngle < (gyroInitial + 180) - 360) {
            gyrochange = -((360 - gyroInitial) + angles.thirdAngle) / runtime.seconds();
        } else if (angles.thirdAngle > gyroInitial + 180) {
            gyrochange = (gyroInitial + (360 - angles.thirdAngle)) / runtime.seconds();
        } else {
            gyrochange = (gyroInitial - angles.thirdAngle) / runtime.seconds();
        }

    }

    public static double readGyro() {

        if(angles.thirdAngle-gyroInitial<-360)
            return (gyrochange * (runtime.seconds())) + angles.thirdAngle-gyroInitial+720;
        else if (angles.thirdAngle-gyroInitial<0)
            return (gyrochange * (runtime.seconds())) + angles.thirdAngle-gyroInitial+360;
        else
            return (gyrochange * (runtime.seconds())) + angles.thirdAngle-gyroInitial;
    }

    public static void resetGyro()
    {
        gyroInitial=angles.thirdAngle;
        runtime.reset();
    }



}
