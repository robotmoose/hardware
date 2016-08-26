//servo_6_star.scad
//servo gear 6 star

//A scad file for a servo end/extension of 6 pegs.
//Units are in milimeters!

use <robo_arm.scad>;

error = 0.5; 

gear_height = 5 + error; 
gear_radius = 5.5 + error; 

peg_width = 9.5 + error; 
peg_depth = 7 + error; 

height = 2 + error; //change this!!!!!!!!!

module servo_end_6_star(middle_radius, height, peg_width, peg_depth){
    
    //Make the inner circle
    cylinder(height, gear_radius, gear_radius);
    
    pegs = 6; 
    angle = 360/pegs; 
    for( i = [0:pegs]){
        rotate([0, 0, angle*i])
              translate([peg_width, 0 , 0 ]) 
                armBase(peg_width, height, peg_depth); 
    }
   // translate([width, 0 , 0 ]) 
    //    armBase(width, height, depth); 
    
}

servo_end_6_star(gear_radius, height, peg_width, peg_depth); 