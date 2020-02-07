/*
Copyright 2019 FIRST Tech Challenge Team 12886

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Servo;
import java.text.DecimalFormat;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import java.text.SimpleDateFormat;
import java.util.Date;
import com.qualcomm.robotcore.hardware.DistanceSensor;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class YooshGyro extends OpMode {
    // CONFIGURATION

    // Expansion Hub 1:
    // Motors:
    // Port 0: manArm
    // Servos
    // Port 0: leHand <-- Still needed
    // Sensors:
    // I2C Bus 0: Expansion Hub IMU
    // I2C Bus 1: frontDistance

    // Expansion Hub 2:
    // Motors:
    // Port 0: frontLeft
    // Port 1: rearLeft
    // Port 2: frontRight
    // Port 3: rearRight

    public DecimalFormat format = new DecimalFormat("##.00");
    private BNO055IMU imu;
    private DcMotor frontLeft;
    private DcMotor rearLeft;
    private DcMotor frontRight;
    private DcMotor rearRight;
    private DcMotor manArm;
    private boolean rightBumperDown = false;
    private boolean leftBumperDown = false;
    private boolean rightStickClick = false;

    private Servo elbow;
    private Servo hand;

    private DistanceSensor frontDistance;

    private String armState = "0block"; // <-- Using string comparators to better explain...
                                        // ...what is happening with the armState

    private String driveState = "step1";

    public double leftStickY;
    public double leftStickX;
    public double rightStickX;

    public Orientation angles;
    public Acceleration gravity;

    private boolean arcadeMode = false;
    private boolean manualControl = false;

    private int gyroCalibratedCount = 0;
    public double scalingFactor = 0.85;
    private float manualArmPower;

    @Override
    public void init() {
        armState = "0block";
        motorInit();
        gyroInit();
        //gamepadInit();
        sensorInit();
        hand.setPosition(0.3);
    }

    @Override
    public void init_loop() {
        gyroLoop();
        // measureDistance();
    }

    @Override
    public void loop() {
        gyroLoop();
        measureDistance();
        setManualMode();
        setDrivePower();
        selectPosition();
        moveArm();
        dropBlock();
        resetArm();
    }

    private void gamepadInit() {
        gamepad1.setJoystickDeadzone(0.2f);
    }

    private void motorInit() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        manArm = hardwareMap.get(DcMotor.class, "manArm");

        hand = hardwareMap.get(Servo.class, "leHand");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        manArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        manArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        manArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("MOTORS", "Initialized");
    }

    private void gyroInit() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);
    }

    private void sensorInit() {
        frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");
    }

    private void gyroLoop() {
        //telemetry.addData("IMU", imu.isGyroCalibrated() ? "Initialized" : "Initializing...");
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //telemetry.addData("ANGLE", angles);
        //telemetry.addData("HEADING", angles.firstAngle);
    }

    // It is possible for buttons to be read multiple times through a loop, triggering
    // actions multiple times unintentionally.  Using a debounce routine fixes this issue
    // by ignoring further presses of a button until it has been released.
    //
    // Pattern:
    // Use a boolean variable to track whether a button has been released yet.
    // boolean x_released = true; // Button is not currently pressed
    // if (gamepad1.x) {  // X Button is pressed
    //     if (x_released) { if the x button is not currently pressed...
    //         x_released = false; // mark the button as not having been released yet
    //         // Do an action here
    //     }
    // }
    // else {
    //     x_released = true;

    private void setManualMode() {
      //telemetry.addData("MANUAL MODE:", manualControl); // <-- See above comment block for explanation
      if(gamepad1.right_bumper) {
        if(!rightBumperDown){
          rightBumperDown = true;
          manualControl = !manualControl;
        }
      }
      else {
        rightBumperDown = false;
      }
    }

    private void dropBlock() {
      if(gamepad1.left_bumper) {
        if(!leftBumperDown){
          leftBumperDown = true;
          hand.setPosition(0.3);
        }
      } else {
        leftBumperDown = false;
      }
    }

    private void resetArm() {
      if(gamepad1.right_stick_button) {
        if(!rightStickClick){
          rightStickClick = true;
          manArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          manArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
      } else {
        rightStickClick = false;
      }
    }

    private void manualMoveArm() {
        manualArmPower = (gamepad1.left_trigger * -1) + (gamepad1.right_trigger);
       // telemetry.addData("MANUAL POWER:", manualArmPower);
        manArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        manArm.setPower(manualArmPower);
    }

    private void selectPosition() {
       //telemetry.addData("ARM STATE:", armState);
        if(gamepad1.a){
          armState = "0block";
        }
        else if (gamepad1.x){
          armState = "1block";
        }
        else if (gamepad1.y){
          armState = "2block";
        }
        else if (gamepad1.b){
          armState = "3block";
        }
    }


    private void moveArm() {
        //telemetry.addData("ManArm", manArm.getCurrentPosition());
        if(manualControl){
          manualMoveArm(); // <-- If manual control mode is active...
                           // ...revert to trigger control.
                           // RightTrigger = up
                           // LeftTrigger = down
          return;          // <-- This will exit the function early so...
                           // ...the state machine doesn't control the arm
        }
        switch(armState){
          case "0block":
          manArm.setTargetPosition(0);
          if(frontDistance.getDistance(DistanceUnit.CM) < 4.0 && !leftBumperDown) {
            hand.setPosition(1);
            if(hand.getPosition() == 1) {
              armState = "1block";
            }
          }
          break;

          case "1block":
          manArm.setTargetPosition(420);
          if(frontDistance.getDistance(DistanceUnit.CM) < 4.0 && !leftBumperDown) {
            hand.setPosition(1);
          }

          break;

          case "2block":
          manArm.setTargetPosition(600);
          if(frontDistance.getDistance(DistanceUnit.CM) < 4.0 && !leftBumperDown) {
            hand.setPosition(1);
          }

          break;

          case "3block":
          manArm.setTargetPosition(880);
          if(frontDistance.getDistance(DistanceUnit.CM) < 4.0 && !leftBumperDown) {
            hand.setPosition(1);
          }

          break;

        }
        manArm.setPower(0.3);
        manArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // moveClaw();
    }

    private void moveClaw() {
      hand.setPosition(0.3);
    }

    private void setDrivePower() {
        final double rotation = -Math.pow(gamepad1.right_stick_x, 3.0) * -0.5;
        final double y = Math.pow(gamepad1.left_stick_y, 3.0) * -0.5;
        final double x = -Math.pow(gamepad1.left_stick_x, 3.0) * -0.5;

        final float currHeading = angles.firstAngle;
        final double direction = Math.atan2(x, y) + (arcadeMode ? currHeading : 0.0);
        // final double direction = Math.atan2(x, y) + 90.0;
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        final float brake = 1.0f;

        final double lf = (speed * Math.sin(direction + Math.PI / 4.0) + rotation) * brake * scalingFactor;
        final double rf = (speed * Math.cos(direction + Math.PI / 4.0) - rotation) * brake * scalingFactor;
        final double lr = (speed * Math.cos(direction + Math.PI / 4.0) + rotation) * brake * scalingFactor;
        final double rr = (speed * Math.sin(direction + Math.PI / 4.0) - rotation) * brake * scalingFactor;

        frontLeft.setPower(-lf);
        frontRight.setPower(-rf);
        rearLeft.setPower(lr);
        rearRight.setPower(rr);
    }

    private void measureDistance() {
        telemetry.addData("FL", frontLeft.getCurrentPosition());
        telemetry.addData("BL", rearLeft.getCurrentPosition());
        telemetry.addData("FR", frontRight.getCurrentPosition());
        telemetry.addData("BR", rearRight.getCurrentPosition());
    }

}
