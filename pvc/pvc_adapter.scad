pipe_od=27;
wiggle=0.5;
hole_d=3;
hole_spacing=55;
walls=3;
flange=[10,4];
floor_h=3;
neck_h=30;
$fn=100;
neck_torus_r=2.7; //CAREFUL WITH THIS!

neck_od=pipe_od+walls*2;
neck_id=pipe_od+wiggle;

module fin(size=[30,3,20])
{
	translate([0,size.y/2,0])
		rotate([0,-90,90])
			linear_extrude(height=size.y)
				polygon(points=[[0,0],[size.z,0],[0,size.x]]);
}

linear_extrude(height=floor_h)
	difference()
	{
		scale([hole_spacing+flange.x*2,pipe_od+walls*2+flange.y*2])
			circle(d=1);
		translate([hole_spacing/2,0])
			circle(d=hole_d);
		translate([-hole_spacing/2,0])
			circle(d=hole_d);
	}

translate([0,0,floor_h])
{
	difference()
	{
		linear_extrude(height=neck_h)
			difference()
			{
				circle(d=neck_od);
				circle(d=neck_id);
			}
		neck_hole_w=pipe_od+walls*2+wiggle;
		translate([0,neck_hole_w/2,neck_h*0.55])
			rotate([90,0,0])
				cylinder(d=hole_d,h=neck_hole_w);
	}
}

translate([-(pipe_od+wiggle)/2,0,0])
	fin([hole_spacing/2-walls-(pipe_od+wiggle)/2,walls,neck_h]);
rotate([0,0,180])
	translate([-(pipe_od+wiggle)/2,0,0])
		fin([hole_spacing/2-walls-(pipe_od+wiggle)/2,walls,neck_h]);

translate([0,0,floor_h])
	rotate_extrude(angle=360)
		translate([neck_od/2,0,0])
			circle(r=neck_torus_r);