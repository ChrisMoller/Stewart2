$fs = .2;
$fa = 2;

shaft_len  = 150;
shaft_wid  =   6;
shaft_thk  =   4;
socket_rad = 6.1;
ball_rad   = 5.0;
socket_off = socket_rad + 2;

module socket() {
   difference () {
     sphere(socket_rad);
     sphere(ball_rad);
     translate([0, 0, socket_rad + 3]) 
        cube(2*socket_rad, center=true);
   }
}

module shaft() {
  rotate([90, 0, 0]) {
    translate([0, -shaft_wid / 2, 0]) 
      cube([shaft_len, shaft_wid, shaft_thk]);
    translate([socket_off, 0, (shaft_thk + socket_rad) - 1]) 
      socket();
    translate([shaft_len - socket_off, 0, (shaft_thk + socket_rad) - 1]) 
      socket();
  }
}

difference() {
  shaft();
  translate([-5, -20, -8]) cube([shaft_len + 10, 40, 8]);
}