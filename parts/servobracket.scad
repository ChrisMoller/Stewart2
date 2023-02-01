$fs = .2;
$fa = 2;

servo_wid   = 12;
servo_dpth  = 20;
servo_hgt   = 25.4;
servo_standoff = 4;
servo_mt_off   = 2;
servo_cc       = 52;

plate_wid   = servo_cc + servo_wid;
plate_dpth  = 20;
plate_hgt   = 34;
plate_thk   = 2;

module bracket() {
  difference() {
    cube([plate_wid,   plate_dpth + plate_thk, plate_hgt]);
    translate([-1,plate_thk, plate_thk])        // main cutout
      cube([plate_wid+2, plate_dpth + plate_thk, plate_hgt]);
    translate([2,-1, servo_standoff+plate_thk]) 
      cube([servo_wid,4,servo_hgt]);  // left servo
    translate([servo_cc-2,-1, servo_standoff+plate_thk]) 
      cube([servo_wid,4,servo_hgt]);  // right servo
    translate([19,-1, servo_standoff+plate_thk]) 
      cube([26,4,servo_hgt]);  // hole
    translate([2,4,-1]) 
      cube([servo_wid, plate_dpth-4, 5]); //left wall hole
    translate([26.5,4,-1]) 
      cube([11, plate_dpth-4, 5]); // ctr floor hole
    translate([servo_cc-2, 4, -1])
      cube([servo_wid, plate_dpth-4, 5]); //right floor hole
    translate([21,  7, -.1]) cylinder(3,1.75,1.75);
    translate([21, 17, -.1]) cylinder(3,1.75,1.75);
    translate([43,  7, -.1]) cylinder(3,1.75,1.75);
    translate([43, 17, -.1]) cylinder(3,1.75,1.75);
  }
}

//translate([0,plate_hgt,0]) rotate([90,0,0]) 
bracket();