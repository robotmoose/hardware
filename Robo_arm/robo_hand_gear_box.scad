//robo_hand_gear_box.scad

use <robo_finger.scad>; 
use <gear.scad>; 
use <servo_6_star.scad>; 

//$fn = 30; 
error = 0.5; 

servo_plus_arm_height = 13 + error; 
servo_width = 23 + error; 
servo_depth = 12 + error; 
finger_height = 9 + error; 

box_height = 2* (servo_plus_arm_height + finger_height); 

pole_height = 20; 
pole_radius = 5; 

module hand_gear_box(box_width, box_depth, box_height, pole_height, pole_radius){
    translate([-box_width/2, -box_depth/2, 0])
        cube([box_width,box_depth,box_height]); 
}

pegs = 16; //how many pegs stick out from the circle
gear_height = 9 + error; //The inner circle height
gear_radius = 30;//(35-6)/2 + 7; 
total_gear_radius = gear_radius + 2; 

box_width = gear_radius*2 + 17; 
box_depth = box_width; 

peg_width = gear_radius + 35; //Make this bigger than the gear_radius
peg_height = gear_height;                   
peg_depth = 5; 

servo_inner_radius = 5.5 + error; 
servo_peg_width = 9.5 + error; 
servo_peg_depth = 7 + error; 
servo_height = 5; 

difference(){
    union(){
//->
difference(){
    hand_gear_box(box_width, box_depth, box_height, pole_height, pole_radius);
    
    translate([-(box_width-10)/2, -(box_depth -10)/2, 5])
        cube([box_width - 10, box_depth - 10, box_height -10]);
    
    rotate([0,0,90])
        translate([-servo_width/2, -servo_depth/2, -1])
            cube([servo_width, servo_depth, servo_plus_arm_height + 1]);
    
    rotate([0,0,90])
        translate([-servo_width/2, -servo_depth/2, box_height-servo_plus_arm_height + 1])
            cube([servo_width, servo_depth, servo_plus_arm_height + 1]);
    
    //Make the openings
    translate([-box_width/2,0,servo_plus_arm_height - 1])
    cube([box_width/2, box_width/2, finger_height + 2]);
    
     
    
    translate([0,0,servo_plus_arm_height+finger_height - 1])
    cube([box_width/2, box_width/2, finger_height + 2]);
}

difference(){
translate([-box_width/2, -box_depth/2 -(servo_height + 2), 0])
    cube([box_width, servo_height + 2, box_height ]); 
translate([0, -box_depth/2 -3 , box_height/2])
    rotate([90,0,0])
        servo_end_6_star(servo_inner_radius, servo_height +1, servo_peg_width, servo_peg_depth);
}
}

translate([-box_depth, -box_depth, box_height/2])
    cube([box_depth * 2,box_depth * 2, box_height]); 

}


translate([0,0,servo_plus_arm_height])
    robo_finger_mark5(); 
translate([0,0,box_height - (servo_plus_arm_height+finger_height)])
    rotate([0,180,0])
        robo_finger_mark5(); 
/*
//eat out the middle
translate([0,0,servo_plus_arm_height])
    cylinder(gear_height, total_gear_radius, total_gear_radius); */

//<-

/*difference(){
    hand_gear_box(box_width, box_depth, box_height, pole_height, pole_radius); 

    union(){
        //the front of the box where the arms come out
        translate([0,0,servo_plus_arm_height])
            robo_finger_mark3(); 
        mirror(1,0,0)
        translate([0,0,servo_plus_arm_height+finger_height])
            robo_finger_mark3(); 

        //The sides of the box where the arms move to open
        rotate([0,0,45])
            mirror(1,0,0)
                translate([0,0,servo_plus_arm_height+finger_height])
                    robo_finger_mark3();  

        rotate([0,0,-45])
            translate([0,0,servo_plus_arm_height])
                robo_finger_mark3(); 

        //The inside circle removal
        translate([0,0,servo_plus_arm_height])
            cylinder(gear_height, total_gear_radius, total_gear_radius);
        translate([0,0,servo_plus_arm_height+finger_height])
            cylinder(gear_height, total_gear_radius, total_gear_radius);
    }
}

*/