// /*
// Copyright 2019 FIRST Tech Challenge Team 12886

// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
// associated documentation files (the "Software"), to deal in the Software without restriction,
// including without limitation the rights to use, copy, modify, merge, publish, distribute,
// sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all copies or substantial
// portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
// NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
// */
// package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.hardware.DigitalChannel;
// import java.util.concurrent.TimeUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
// import java.text.DecimalFormat;
// import com.qualcomm.hardware.bosch.BNO055IMU;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.hardware.DistanceSensor;
// import com.qualcomm.robotcore.hardware.TouchSensor;

// import java.text.SimpleDateFormat;
// import java.util.Date;

// /**
//  * This file contains an example of an iterative (Non-Linear) "OpMode".
//  * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
//  * The names of OpModes appear on the menu of the FTC Driver Station.
//  * When an selection is made from the menu, the corresponding OpMode
//  * class is instantiated on the Robot Controller and executed.
//  *
//  * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
//  * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
//  */
// @Autonomous

// public class AutoTest extends OpMode {
//     /* Declare OpMode members. */
//     public DecimalFormat format = new DecimalFormat("##.00");
//     private BNO055IMU imu;
//     private DcMotor frontLeft;
//     private DcMotor rearLeft;
//     private DcMotor frontRight;
//     private DcMotor rearRight;

//     private DistanceSensor frontDist;
//     private DistanceSensor leftDist;
//     private DistanceSensor rightDist;

//     public TouchSensor touchSensorboiii;

//     public float gyro_degrees;

//     public int autoState = 1;

//     public double newForward;
//     public double newStrafe;
//     public double newRotation;

//     // State used for updating telemetry
//     public Orientation angles;
//     public Acceleration gravity;

//     private boolean arcadeMode = false;
//     private int gyroCalibratedCount = 0;

//     @Override
//     public void init() {
//         sensorInit();
//         motorInit();
//         gyroInit();

//         telemetry.addData("Status", "Initialized");
//     }

//     private void sensorInit() {
//         frontDist = hardwareMap.get(DistanceSensor.class, "frontDist");
//         leftDist = hardwareMap.get(DistanceSensor.class, "leftDist");
//         rightDist = hardwareMap.get(DistanceSensor.class, "rightDist");
//         touchSensorboiii = hardwareMap.get(TouchSensor.class, "touchSensorboiii");

//         telemetry.addData("SENSORS", "Initialized");

//     }

//     private void motorInit() {
//         frontLeft = hardwareMap.dcMotor.get("frontLeft");
//         rearLeft = hardwareMap.dcMotor.get("rearLeft");
//         frontRight = hardwareMap.dcMotor.get("frontRight");
//         rearRight = hardwareMap.dcMotor.get("rearRight");

//         // frontLeft.setDirection(DcMotor.Direction.REVERSE);
//         // rearRight.setDirection(DcMotor.Direction.REVERSE);
//         rearRight.setDirection(DcMotor.Direction.REVERSE);
//         frontRight.setDirection(DcMotor.Direction.REVERSE);

//         frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//         // frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         // frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         // rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         // rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//         // frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//         // frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//         // rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//         // rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//         telemetry.addData("MOTORS", "Initialized");
//     }

//     private void gyroInit() {
//         imu = hardwareMap.get(BNO055IMU.class, "imu");
//         BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

//         parameters.mode                = BNO055IMU.SensorMode.IMU;
//         parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//         parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//         parameters.loggingEnabled      = false;

//         imu.initialize(parameters);
//     }
//     /*
//      * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
//      */
//     @Override
//     public void init_loop() {
//         autoState=1;
//     }

//     /*
//      * Code to run ONCE when the driver hits PLAY
//      */
//     @Override
//     public void start() {
//     //     if (frontDist.getDistance(DistanceUnit.CM) > 22){
//     //     frontLeft.setPower(0.5);
//     //     frontRight.setPower(0.5);
//     //     rearLeft.setPower(0.5);
//     //     rearRight.setPower(0.5);
//     // }
//     //     else {
//     //         frontLeft.setPower(0);
//     //         frontRight.setPower(0);
//     //         rearLeft.setPower(0);
//     //         rearRight.setPower(0);
//     //         // wait.getTime(timeUnit.SECOND = 10)();
//     //         // frontLeft.setPower(-0.5);
//     //         // rearLeft.setPower(0.5);
//     //         // frontRight.setPower(0.5);
//     //         // rearRight.setPower(-0.5);
//     //       //make if robo is stopped for ten seconds strafe


//     //     }
//     //     frontLeft.setTargetPosition(2000);
//     //     rearLeft.setTargetPosition(2000);
//     //     frontRight.setTargetPosition(2000);
//     //     rearRight.setTargetPosition(2000);

//         // frontLeft.setTargetPosition(-2000);
//         // rearLeft.setTargetPosition(-2000);
//         // frontRight.setTargetPosition(-2000);
//         // rearRight.setTargetPosition(-2000);
//     }




//     /*
//      * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
//      */
//     @Override
//     public void loop() {
//         telemetry.addData("FrontDist", String.format("%.01f cm", frontDist.getDistance(DistanceUnit.CM)));
//         telemetry.addData("LeftDist", String.format("%.01f cm", leftDist.getDistance(DistanceUnit.CM)));
//         telemetry.addData("RightDist", String.format("%.01f cm", rightDist.getDistance(DistanceUnit.CM)));
//         telemetry.addData("touchSensorboiii", touchSensorboiii.isPressed());


//         //Go forward at 0.5 checking if objects are 22 centimeters away
//           //If object close enough move to case 2
//         switch (autoState){
//             case 1:
//                 frontLeft.setPower(0.5);
//                 frontRight.setPower(0.5);
//                 rearLeft.setPower(0.5);
//                 rearRight.setPower(0.5);
//                 if (frontDist.getDistance(DistanceUnit.CM) < 22) {
//                     autoState = 2;
//                 }
//             break;



//            //If button pressed then move backwards and pull up arm
//             case 2:
//                 frontLeft.setPower(0);
//                 frontRight.setPower(0);
//                 rearLeft.setPower(0);
//                 rearRight.setPower(0);
//                // manArm.setPower(0.2)
//                 if (touchSensorboiii.isPressed() == true) {
//                      autoState = 3;
//                 }
//             break;

//           //go backwards until the dist sensor reads further than 22 cm in which case, go to case 4
//             case 3:
//                 frontLeft.setPower(-0.5);
//                 frontRight.setPower(-0.5);
//                 rearLeft.setPower(-0.5);
//                 rearRight.setPower(-0.5);
//                 //manArm.setPower(-0.2)
//                 if (frontDist.getDistance(DistanceUnit.CM) > 22){
//                     autoState = 4;
//                 }
//             break;

//             //stop
//             case 4:
//                 frontLeft.setPower(0);
//                 frontRight.setPower(0);
//                 rearLeft.setPower(0);
//                 rearRight.setPower(0);

//             break;



//      }
//     }



//     /*
//      * Code to run ONCE after the driver hits STOP
//      */
//     @Override
//     public void stop() {
//         frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//         autoState = 1;
//     }
// }
