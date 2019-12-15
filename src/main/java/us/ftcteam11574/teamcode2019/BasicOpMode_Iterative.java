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

package us.ftcteam11574.teamcode2019;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Game OpMode", group="Iterative Opmode")
//@Disabled
public class BasicOpMode_Iterative extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor tl = null; //top left
    private DcMotor tr = null; //top right
    private DcMotor bl = null; //bottom left
    private DcMotor br = null; //bottom right
    private DcMotor intakeR = null;
    private DcMotor intakeL = null;
    private DcMotor pantagraph = null;
    Rev2mDistanceSensor mDistanceSensor;
    ColorSensor mColorSensor;
    BNO055IMU imu;
    private DcMotor[] motors; //all motors

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        tl  = hardwareMap.get(DcMotor.class, "tl");  //45 degrees //v0
        tr  = hardwareMap.get(DcMotor.class, "tr");  //135 degrees //v1
        bl  = hardwareMap.get(DcMotor.class, "bl");  //225 degrees //v2
        br  = hardwareMap.get(DcMotor.class, "br");  //315 degrees //v3
        intakeR  = hardwareMap.get(DcMotor.class, "intakeR");  //225 degrees //v2
        intakeL  = hardwareMap.get(DcMotor.class, "intakeL");  //315 degrees //v3
        pantagraph  = hardwareMap.get(DcMotor.class, "pant");  //315 degrees //v3
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        tl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeL.setDirection(DcMotorSimple.Direction.FORWARD);

        gamepad1.setJoystickDeadzone(.05f);
        mDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance");
        mColorSensor = hardwareMap.colorSensor.get("color");




        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery


        //strategy for fixing
        //first, run all forward using the teleop mode
        //If its working, should look like this (game pad a)
        //          //test this last, to make sure it actually moves correctly
        //          V / \ ^
        //          V \ / ^
        //         game pad b //should test this one out first, since all are different speeds, use this to
        //          V / \ ^   //Can't use just this though,
        //         VV \ / ^^
        //         game pad x //test this next (see notebook for what it will look like if its wrong
        //         ^^^ / \ V
        //           V \ / ^^^
        tl.setDirection(DcMotor.Direction.FORWARD);
        tl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tr.setDirection(DcMotor.Direction.FORWARD);
        tr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setDirection(DcMotor.Direction.FORWARD);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors = new DcMotor[]{br,tr,tl,bl}; //

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    boolean max_mode = true;
    boolean a_cur = false;
    boolean orientation_mode = false;
    boolean b_cur = false;
    boolean x_cur = false;
    boolean slow_intake = false;
    @Override
    public void loop() {


        double vx = -gamepad1.left_stick_x;
        double vy = gamepad1.left_stick_y;
        double rot = -gamepad1.right_stick_y;
        double rot2 = gamepad1.right_stick_x;


        if(gamepad1.a){
            if ( !a_cur ) {
                a_cur = true;
                max_mode = !max_mode;
            }

        }
        else {
            if (a_cur) {
                a_cur = false;
            }
        }
        if(gamepad1.b){
            if ( !b_cur ) {
                b_cur = true;
               orientation_mode = !orientation_mode;
            }

        }
        else {
            if (b_cur) {
                b_cur = false;
            }
        }

        if(gamepad1.x){
            if ( !x_cur ) {
                x_cur = true;
                slow_intake= !slow_intake;
            }

        }
        else {
            if (x_cur) {
                x_cur = false;
            }
        }
        double div = 1;
        if (slow_intake) {
            div = 4;
        }
        //test if block already there
        telemetry.addData("Block found?:",isBlock());
        if(!isBlock()) {
            intakeR.setPower((gamepad1.left_trigger - gamepad1.right_trigger) / div);
            intakeL.setPower((gamepad1.left_trigger - gamepad1.right_trigger) / div);

        }


        pantagraph.setPower( ((gamepad1.left_bumper ? 1 : 0) -(gamepad1.right_bumper ? 1 : 0 )) );



        //test cases
        /*
        if(gamepad1.a){ //test forwards
            telemetry.addData("Motors", "v0 (%.2f), v1 (%.2f), v2 (%.2f), v3 (%.2f)",1f,1f,1f,1f);
            setMotors(new double[]{1,1,1,1},1);
        }
        else if(gamepad1.b){ //1 .3 -.3 -1
            telemetry.addData("Motors", "v0 (%.2f), v1 (%.2f), v2 (%.2f), v3 (%.2f)",1f,.3f,-.3f,-1f);
            setMotors(new double[]{1,.3,-.3,-1},1);
            //new DcMotor[]{br,tr,tl,bl};
            //br = -.3
            //bl = .3
            //tr =-1
            //tl - 1
            //{tl,bl,br,tr}

        }
        else if(gamepad1.x){ //test
            telemetry.addData("Motors", "v0 (%.2f), v1 (%.2f), v2 (%.2f), v3 (%.2f)",1f,-1f,1f,-1f);
            setMotors(new double[]{1,-.3,1,-.3},1 );
        }
         */
        telemetry.addData("Orietnation_mode on?:", orientation_mode);
        telemetry.addData("test_mode on?:", slow_intake);
        telemetry.addData("Max_mode on?:", max_mode);
        if(!orientation_mode) {

            setMotors(-vx,vy,rot2*1.5);
            // Show the elapsed game time and wheel power.
            //telemetry.addData("Motors", ("Vx: " + -vx + "----Vy" + vy + "---Rot2" + rot2));
            double[] powers = motorPower.calcMotorsFull(vx, vy, rot2);//can also calc max, which always goes the fastest
            //telemetry.addData("Motors", "v0 (%.2f), v1 (%.2f), v2 (%.2f), v3 (%.2f)", powers[0], powers[1], powers[2], powers[3]);
            //telemetry.addData("First ang", "Ang1:" + imu.getAngularOrientation().firstAngle);
        }
        else {
            rot2 = -rot2;
            double ang = Math.atan2(-vy,vx);
            double mag = Math.sqrt(vy*vy + vx*vx);
            //atan works like this
            //         90
            //        ^
            // +-180  <   > 0
            //      -90 V
            //having some trouble with resetting
            //When both 0, shoudl be equal to -90
            //At 0, as if 90, so goes forward (CHECK)
            //At 90, as if
            double imu_ang =(imu.getAngularOrientation().firstAngle + Math.PI/2.0) % Math.PI;
            double goal_ang = (ang - (-imu_ang)) + Math.PI/2.0 ; //This needs to be offset by some number of degrees, because when both are zero, it should move foward
            double ang_rot = Math.atan2(rot,rot2); //angle your game stick is facing
            double mag_rot = Math.sqrt(rot*rot + rot2*rot2);
            double goal_rot_ang = distanceTurn(-imu_ang,ang_rot);
            double nrot = 0;
            double nvx = Math.cos(goal_ang) * mag;
            double nvy = Math.sin(goal_ang) * mag;
            if(Math.abs(goal_rot_ang) > 0.10) { //amount goal angle needs to be off to actually turn in that direction
                nrot = goal_rot_ang * (mag_rot)  ; //rotation is the direction of rotation
            }
            else {
               nrot = goal_rot_ang * (mag_rot)/3.0  ; //rotation is the direction of rotation
            }
            //-90first angle to 180 goal angle
            setMotors(nvx,nvy,nrot);
            //only going from -90 to 90 for some reason, getting to zero at two different places
            double[] powers = motorPower.calcMotorsFull(nvx, nvy, nrot);//can also calc max, which always goes the fastest
            //telemetry.addData("Ang of right stick",ang * 180/Math.PI);
            //telemetry.addData("Ang of left stick",ang_rot * 180/Math.PI); //wait, how did this get to 270?
            telemetry.addData("Info", ("Goal rot _ang: " + goal_rot_ang * 180/Math.PI));
            telemetry.addData("Motors", ("Vx: " + nvx + "----Vy" + nvy + "---Rot" + nrot));
            telemetry.addData("Motors", "v0 (%.2f), v1 (%.2f), v2 (%.2f), v3 (%.2f)", powers[0], powers[1], powers[2], powers[3]);
            telemetry.addData("First ang", "Ang1:" + (-imu.getAngularOrientation().firstAngle) * 180/Math.PI );
            //I think it swaps at 180 instead of 360 on the angle orientation mode with first angle
        }
        //assuming that this returns as its supposed to
        //imu.getAngularVelocity();
        //imu.getAcceleration();
        //imu.getPosition(); //coudl use this to check if everythign isin the eiright position during autonomous
        //use these to check how close to the goal the robot is actually achieving



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    @Override
    public void stop() {
    }
    public boolean isBlock() {
        final int[] block = new int[]{255,255,0};
        int[] color_read = new int[]{mColorSensor.red(), mColorSensor.green(), mColorSensor.blue()};
        //now meansure the distance between the colors, weight blue most highly, since yellow shouldn't have nearly any blue
        int dist = Math.abs(block[0]-color_read[0]) + Math.abs(block[1]-color_read[1]) + (Math.abs(block[2]-color_read[2])*5);
            //blue is weighted most strongly, 5 times as important for now

        if (dist < 70) { //increasing this number will increase the sensitivity

            return true;

        }
        telemetry.addData("color",
        String.format("r:%d, g:%d, b:%d, a:%d",
                mColorSensor.red(),
                mColorSensor.green(),
                mColorSensor.blue(),
                mColorSensor.alpha()));

        telemetry.addData("distance (mm)", mDistanceSensor.getDistance(DistanceUnit.MM));
        telemetry.update();

        return false;

    }
    public void setMotors( double vx, double vy, double rot) {
        double[] powers = motorPower.calcMotorsFull(vx, vy, rot);
        if (max_mode) { //max speed always,maximym possible speed
            setMotorsMax(powers, 1);
        } else {
            //1.6, constant near where this reaches its max, (on the lower side, to ensure it doens't go over)
            for (int i = 0; powers.length > i; i++) {
                powers[i] *= 1.6;
            }
            setMotors(powers, max_abs(new double[]{Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2)), rot}));
        }
    }


    public void setMotors(double[] powers, double len) {
        //powers motors with given ratios, and rescales so the max length is len.
        double max = max_abs(powers);
        double len_mult = 1;
        //does't chnage the motor speed unless it has to
        if (max > 1) {
                len_mult = len / max;
        }
        //telemetry.addData("MAX", "max " + max);
        //telemetry.addData("Power", "len_mult " + len_mult);
        for (int i = 0; powers.length > i; i++) {
           // telemetry.addData("Power motors",  powers[i] * len_mult);
            motors[i].setPower(powers[i] * len_mult);
        }

    }
    public void setMotorsMax(double[] powers, double len) {
        //powers motors with given ratios, and rescales so the max length is len.
        double max = max_abs(powers);
        double len_mult = 1;


        if (Math.abs(max) < 0.01) {
            len_mult = 0;
        }
        else {
            len_mult = len / max;
        }

        //telemetry.addData("MAX", "max " + max);
        //telemetry.addData("Power", "len_mult " + len_mult);
        for (int i = 0; powers.length > i; i++) {
            motors[i].setPower(powers[i] * len_mult);
        }

    }

    public double max_abs(double[] list) {
        double max = Math.abs(list[0]);
        //int id = 0;
        for (int i = 1; list.length > i; i++) {
           if (Math.abs(list[i]) > max) {
               max = Math.abs(list[i]);
               //id = i;
           }
        }
        return max;

    }
    public double atan2(double y, double x) {
        if(x==0) {
            if (y > 0) {
                return Math.PI/2.0;
            }
            else {
                return 3*Math.PI/2.0;
            }
        }
        return Math.atan(y/x)+((((Math.abs(x)/x)-1)/2) * Math.PI);
    }

    public double distanceTurn(double cur, double goalAngle) {
        //The way to calculate this is as follows:
            //if goalAngle is less than 0, than add 360
            //so angles from (-180,0] get turned into (180,360]
            //angles from (0,180] stay as (0,180]
        //then we can just find which is a shorter distance

        //then we need to convert the current angle to the zero to 360 thing
        //now compare the distance
        double ncur = convertAng(cur);         //90
        double ngoal = convertAng(goalAngle);  //270
        double dif = (ngoal-ncur);              //179 , 181
        //we have a problem when going straight backwards
        //it treats it the same as going straight forwards
        //and it thinks were much closer than we are it seems
        //lets consider the case of the cur=90 goal = -90 -> 270
        //dif = 180
        //dif = 360-180 = 180 (good)
        if (dif >= Math.PI) {
            return -((Math.PI*2)-dif);  //need to decrease angle
            //
        }
        else if (dif <= -Math.PI) {
            return(( Math.PI*2)+dif); //increase angle
        }
        else if(dif > 0 && dif < Math.PI) {
            return dif;
        }
        else if(dif < 0 && dif > -Math.PI) {
            return dif;
        }
        else {
            return 0;
        }



    }
    private double convertAng(double ang) {
        //in the form [-180 to 180]
        if(ang < 0) {
            ang += Math.PI*2;
        }
        return ang;
    }





}
