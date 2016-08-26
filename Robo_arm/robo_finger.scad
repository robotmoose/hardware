//robo_finger.scad
//Laurin Fisher
//June 23, 2016

//Makes a finger for a robot arm. Can be printed twice to get a robot hand. (Units are in Milimeters!)
use <robo_arm.scad>; 
use <gear.scad>; 

module robo_finger_mark5(){
error = 0.5; 

//Make the Gear
servo_arm_x_depth = 6 + error; 
servo_arm_x_width = 35 - servo_arm_x_depth + error; 

servo_arm_radius = 4 + error;

servo_arm_y_depth = 5 + error; 
servo_arm_y_width = 19 + - servo_arm_y_depth + error;

pegs = 16; //how many pegs stick out from the circle
gear_height = 4 + error; //The inner circle height 9
servo_arm_height = gear_height + 50; 
gear_radius = servo_arm_x_width/2 + 7; 

peg_width = gear_radius + 35; //Make this bigger than the gear_radius
peg_height = gear_height;                   
peg_depth = 5; 

//MAKE THE FINGER
finger_height = gear_height; //height of whole object

finger_bottom_width = 35 + gear_radius; 
finger_bottom_depth = 20; 
finger_bottom_angle = 60; 

finger_top_width = 30 + gear_radius; 
finger_top_depth = 20; 

//Build the mini-servo's arm 
module mini_servo_4_arm(width_x, height, depth_x, circle_radius, width_y, depth_y){
    cylinder(height, circle_radius, circle_radius); 
    armBase(width_x, height, depth_x); 
    
    rotate([0,0,90])
        armBase(width_y, height, depth_y); 
}
rotate([0,180,0]){
    translate([0,0,-finger_height]){
union(){
    difference(){
    gear(gear_height, gear_radius, 0, peg_width, peg_depth, gear_height); 
    translate([0,0,-0.5])
        mini_servo_4_arm(servo_arm_x_width, servo_arm_height, servo_arm_x_depth, servo_arm_radius, servo_arm_y_width, servo_arm_y_depth); 
}
difference(){
union(){
    //the base of the finger
    translate([gear_radius,0,0])
        rotate([0,0, 70])
            cube([finger_bottom_width*1.5, finger_bottom_depth, finger_height], 0); 

    //the top part of the finger
    translate([gear_radius*2+.1,finger_bottom_width, 0])
        rotate([0,0,120])
            cube([finger_top_width, finger_top_depth, finger_height] );
    
    //the top part of the finger
    translate([gear_radius*2+finger_top_depth*2,finger_bottom_width, -finger_height*2-1])
        rotate([0,0,120])
            cube([finger_top_width*2, finger_top_depth*2, finger_height*4] );
    
    translate([0, 90, -finger_height])
    cube([finger_top_depth,35,finger_height*2],0); 
}
        //the top part of the finger
    translate([gear_radius*2+finger_top_depth*2,finger_bottom_width, -finger_height*2-1])
        rotate([0,0,120])
            cube([finger_top_width*2, finger_top_depth*2, finger_height*4] );
    
}
}
}
}
}

robo_finger_mark5(); 