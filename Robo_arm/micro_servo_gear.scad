//micro_servo_gear.scad
//Laurin Fisher
//July 28, 2016

//Makes a generic gear. The code can be copied then modified for customized use. UNITS = MILIMETERS

use <robo_arm.scad>; 

$fn = 30; 
error = 0.5; 

//Build the mini-servo's arm 
module mini_servo_4_arm(width_x, height, depth_x, circle_radius, width_y, depth_y){
    cylinder(height, circle_radius, circle_radius); 
    armBase(width_x, height, depth_x); 
    
    rotate([0,0,90])
        armBase(width_y, height, depth_y); 
}

servo_arm_height = 15; 
servo_arm_radius = 4 + error; 

servo_arm_x_depth = 6 + error; 
servo_arm_x_width = 35 - servo_arm_x_depth + error; 

servo_arm_y_depth = 5 + error; 
servo_arm_y_width = 19 + - servo_arm_y_depth + error;

//Make the Gear

pegs = 16; //how many pegs stick out from the circle
gear_height = 4 + 5 + error; //The inner circle height
gear_radius = servo_arm_x_width/2 + 7; 

peg_width = gear_radius + 35; //Make this bigger than the gear_radius
peg_height = gear_height; 
peg_depth = 5; 


//Module to make a gear with rounded edges using armBase()
module gear(gear_height, gear_radius, pegs, peg_width, peg_depth, peg_height){
    
    //The cylinder size (inner circle)
    cylinder(gear_height, gear_radius, gear_radius); 

    //The pegs, each distributed evenly over the 360 deg
    angle = 360/pegs; 
    for( i = [0:pegs]){
        rotate([0, 0, angle*i])
            armBase(peg_width, peg_height, peg_depth); 
            //comment armBase and uncomment line below for non-rounded gear pegs
            //cube([peg_width, peg_depth, peg_height], 0); 
    }
}

difference(){
    gear(gear_height, gear_radius, pegs, peg_width, peg_depth, peg_height); 
    
    translate([0,0,2])
        mini_servo_4_arm(servo_arm_x_width, servo_arm_height, servo_arm_x_depth, servo_arm_radius, servo_arm_y_width, servo_arm_y_depth); 
}
