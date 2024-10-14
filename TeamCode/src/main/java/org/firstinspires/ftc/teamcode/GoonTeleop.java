/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="GoonTeleop", group="Iterative OpMode")

public class GoonTeleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor lift = null;
    private DcMotor turnTable = null;
    private CRServo claw = null;
    private Servo extender = null;
    private CRServo caraSpin = null;
    private CRServo servoIntake = null;
    PController liftPController = new PController(0.002);




    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotor.class, "left_drive_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_drive_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_drive_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_drive_back");

        lift = hardwareMap.get(DcMotor.class, "lift");
        turnTable = hardwareMap.get(DcMotor.class, "turn_table");

        claw = hardwareMap.get(CRServo.class, "claw");
        caraSpin = hardwareMap.get(CRServo.class, "caraSpin");
        extender = hardwareMap.get(Servo.class, "extender");
        servoIntake = hardwareMap.get(CRServo.class, "servoIntake");


        double minPowerLift = 0;
        double maxPowerLift = 1;
        double liftMaxTicks = 1200;
        int liftPosition = 0;

        liftPController.setInputRange(0, liftMaxTicks);
        liftPController.setSetPoint(0);
        liftPController.setOutputRange(minPowerLift, maxPowerLift);
        liftPController.setThresholdValue(5);






        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);





        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        //toggles
        boolean intakeToggle = false;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        previousGamepad1 = currentGamepad1;
        currentGamepad1 = gamepad1;
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;


        double minPowerLift = 0;
        double maxPowerLift = 1;
        double liftMaxTicks = 2500;
        int safeTicks = 900;
        int liftPosition = 0;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        leftFront.setPower(leftPower);
        leftBack.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightFront.setPower(rightPower);



        //pcontroller lift
        liftPosition = lift.getCurrentPosition();

        if (gamepad1.dpad_up && liftPosition < liftMaxTicks) {   ///move lift up and sets controller position

            lift.setPower(.5);
            liftPController.setSetPoint(liftPosition);

        } else if (gamepad1.dpad_down && liftPosition > 30) {  //move lift down and sets controller position

            lift.setPower(-.5);
            liftPController.setSetPoint(liftPosition);

        } else {                                       //uses proportional controller to hold lift in correct spot
            if (liftPosition < liftPController.setPoint) {

                lift.setPower(minPowerLift +
                        liftPController.getComputedOutput(liftPosition));
            } else {
                lift.setPower(minPowerLift -
                        liftPController.getComputedOutput(liftPosition));
            }
        }



        //turntable
        if(gamepad1.dpad_left && liftPosition > safeTicks)
        {
            turnTable.setPower(0.5);
        }
        else if(gamepad1.dpad_right && liftPosition > safeTicks)
        {
            turnTable.setPower(-0.5);
        }
        else
        {
            turnTable.setPower(0);
        }

        //claw

        if(gamepad1.a)
        {
            claw.setPower(-0.85);
        }
        else if(gamepad1.b)
        {
            claw.setPower(0.85);
        }
        else
        {
            claw.setPower(0);
        }

        //carousel spinner
        if(gamepad1.x)
        {
            caraSpin.setPower(0.85);
        }
        else
        {
            caraSpin.setPower(0);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Lift", lift.getPower());
        telemetry.addData("setpt", liftPController.setPoint);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
