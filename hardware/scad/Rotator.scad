include <BOSL2/std.scad>
include <BOSL2/ball_bearings.scad>
include <BOSL2/screws.scad>

$fn=256;
extension_tube_outer_diameter = 44.5;
extension_tube_inner_diameter = 42;
bearing_outer_diameter = 72.4;
bearing_inner_diameter = 55.2;
bearing_height = 9;
no_of_clips = 3;
motor_distance = 75.8;  // calculated optimal distance of motor in x direction dependent on the used beltlength
cover_length = 50;      // length of motor cover - 20, so this is 70 long in fact
cover_width = 26;       // width of motor cover - 20, so this is 46 wide in fact
with_supports = true;
with_bearing_clips = false;


module esp_clip(){
    difference() {
        hull(){
            translate([0,-10.5,0])cylinder(d=4, h=2);
            translate([0, 10.5,0])cylinder(d=4, h=2);
        }
        translate([0,-10.5,-1])cylinder(d=2.5, h=4);
        translate([0, 10.5,-1])cylinder(d=2.5, h=4);
    }
}

module tmc_clip(){
    difference(){
        union(){
            hull(){
                translate([0,-9,0])cylinder(d=4, h=2);
                translate([0, 9,0])cylinder(d=4, h=2);
            }    
            hull(){
                translate([-2,-2,0])cylinder(d=4, h=2);
                translate([-2, 2,0])cylinder(d=4, h=2);
            }  
        }
        translate([0,-9,-1])cylinder(d=2.5, h=4);
        translate([0, 9,-1])cylinder(d=2.5, h=4);
    }
}

module sensor_clip(){
    difference(){
        cuboid([4,8,2], anchor=BOTTOM);
        translate([0,0,-1])cylinder(d=2.5, h=4);
    }
    translate([2,0,0])cuboid([2,8,6], anchor=BOTTOM+LEFT);
}

module hall_clip() {
    difference() {
        diff() prismoid([7,10-0.8],[7,5.65-0.8], height=2.6, anchor=BOTTOM+LEFT) edge_profile(TOP+RIGHT, excess=2)mask2d_roundover(h=1); 
        translate([1.6,0,-1])cuboid([3.4,4.5,3], anchor=BOTTOM+LEFT);
        translate([-1,0,-1])cuboid([4,3.6,2], anchor=BOTTOM+LEFT);
    }   
}

module distance_ring(h=5) {
    difference(){
        union(){
            // tube with outer diameter and thread with inner diameter
            cylinder(d=extension_tube_outer_diameter, h=h);
            translate([0,0,h]) threaded_rod(d=extension_tube_inner_diameter, pitch=0.75, h=6);
        }
        // minus the bore of the tube itself
        translate([0,0,-1])cylinder(d=extension_tube_inner_diameter-2, h=h+4);
    }
}
  
module rotating_ring(){
    difference(){
        // pulley with dome
        union(){
            import("Imports/BigPulley.obj");
            translate([0,0,5.2+2.2])cyl(h=5, d1=94, d2=94-6, rounding1=-2, rounding2=2);
           
        }
        // hole for the extension tube
        translate([0,0,-1])cyl(d=extension_tube_outer_diameter+0.2, h=10.9, chamfer2=-0.6, anchor=BOTTOM);
        // carve out inner part
        translate([0,0,8.6/2])cyl(h=8.6, d=bearing_outer_diameter+13, rounding1=-1, rounding2=1);
        // hole for the index magnet
        translate([-(bearing_outer_diameter+13)/2-3.6,0,4.1])yrot(90)cyl(d=6.3, h=4, chamfer1=-0.4, anchor=BOTTOM);

        
    }
    // inner cylinder connected to inner ring of the bearing
    difference(){
        // cylinder
        translate([0,0,-1])cylinder(d=bearing_inner_diameter, h=9.7);
        // minus the hole for the extension tube
        translate([0,0,-1])cylinder(d=extension_tube_outer_diameter+0.2, h=12); 
        // minus the 45deg phase
        //translate([0,0,0])cylinder(h=(bearing_inner_diameter-extension_tube_outer_diameter)/2, d1=bearing_inner_diameter, d2=extension_tube_outer_diameter);
        translate([0,0,-1.4])cyl(d=bearing_inner_diameter-2, h=6.5, rounding2=(bearing_inner_diameter-extension_tube_outer_diameter)/2, anchor=BOTTOM);
        // trim to height
        translate([0,0,-1])cylinder(h=1, d=55);
        // cutouts for bearing clips
        if(with_bearing_clips)  
            for (w=[0:360/no_of_clips:360-360/no_of_clips])rotate([0,0,w])translate([-bearing_inner_diameter/2-2,-3.5, -2]) cube([4,7,11]);
    }
    // bearing seat
    difference(){
        translate([0,0,7.6+0.2])cylinder(d=bearing_inner_diameter+5, h=1.2);
        translate([0,0,6])cylinder(d=bearing_inner_diameter-5, h=5);
    }
    // tube stop ring
    difference() {
        union(){
            #translate([0,0,5.0])cylinder(d1=extension_tube_outer_diameter+2,d2=extension_tube_outer_diameter-3, h=1.4);
            translate([0,0,5.0+4.7/2])threaded_rod(d=extension_tube_inner_diameter+0.2, pitch=0.75, h=4.7);
            translate([0,0,9.5])cylinder(d=extension_tube_inner_diameter-1, h=0.2);
            }
        translate([0,0,2.0])cylinder(d=extension_tube_inner_diameter-2.4, h=10);
    }

    // bearing clips
    if(with_bearing_clips){
        for (w=[0:360/no_of_clips:360-360/no_of_clips])rotate([0,0,180 + w + 180 * 5 / bearing_outer_diameter / PI])
        difference(){
            translate([0,0,-1.6])cyl(d=bearing_inner_diameter, h=9.4, rounding1=-1, anchor=BOTTOM);
            translate([0,0,-2])pie_slice(h = 12, d=bearing_inner_diameter+3, ang = 360-360 * 5 / bearing_outer_diameter / PI, anchor=BOTTOM);
            translate([0,0,-3])cyl(h=10.5, d1=bearing_inner_diameter-2, d2=bearing_inner_diameter-2, rounding2=1, anchor=BOTTOM);
            translate([0,0,-3])cylinder(h=13, d=bearing_inner_diameter-6);
        }
    }
    // eventually print support ring
    if (with_supports) translate([0,0,9.9-1.4])difference(){
        cyl(d=105, h=1.4, anchor=BOTTOM);
        translate([0,0,-0.1])cylinder(d=91, h=2);
    } 
    if (false) translate([0,0,9.9-1.6])difference(){
        cyl(d=extension_tube_inner_diameter, h=1.6, anchor=BOTTOM);
        translate([0,0,-0.1])cylinder(d=extension_tube_inner_diameter-8, h=2);
        }
}  

module small_pulley(){
    difference(){
        import("Imports/SmallPulley.obj");
        translate([0,0,-1])cyl(d=5.3, h=9.2, chamfer2=-1, anchor=BOTTOM, $fn=64);
        // magnet
        translate([0,0,-1])cylinder(d=6.3, h=3.1,   $fn=64);
        // room for sensor
        translate([0,0,-1])cylinder(d=8,   h=1.8, $fn=64);
    }
    // add index D-Shape
    rotate([0,0,-90])translate([2.1,-2.5,2.5]) cube([0.7,5,4]);  
}

module mounting_post(){
    difference(){
        translate([0,0,6.2])cyl(d=8, h=12.4, rounding1=-2); 
        translate([0,0,3.5+12.4-7])cyl(d=4, h=7, chamfer2=-0.4);
        translate([0,0,2])cylinder(d=3.3, h=12);
    }   
}

// bottom housing
module bottom (){
    difference(){
        // bottom plate
        union(){
            hull(){
                translate([0,0,-1])cyl(d=140, h=2, chamfer1=1);
                translate([-90,-60,-1])cyl(d=20, h=2, chamfer1=1);
                translate([-90, 60,-1])cyl(d=20, h=2, chamfer1=1);
                
            }
            // tube foot
            cylinder(h=5, d1=49, d2=45);
            // tube main cylinder where the thread gets cut out
            translate([0,0,2])cyl(h=4, d=47, rounding2=1, anchor=BOTTOM);
        }
        // hole for extension tube
        translate([0,0,-2])cyl(d=extension_tube_outer_diameter, h=6, chamfer1=-1, anchor=BOTTOM);
        // thread for extension tube
        translate([0,0,3])threaded_rod(d=extension_tube_inner_diameter+0.6, pitch=0.75, h=6);
        // pocket for rotation sensor
        translate([-motor_distance,0,-1.4])cuboid([23.6,23.8,2], rounding=3.2, edges=[FWD+RIGHT,FWD+LEFT,BACK+RIGHT,BACK+LEFT], anchor=BOTTOM); //rounding=3.5
        // cutout for hall sensor
        translate([-bearing_outer_diameter/2-2,0,-1])cuboid([20,5.65,2], chamfer=-1, edges=[BOTTOM+FRONT,BOTTOM+BACK], anchor=BOTTOM+RIGHT);
        translate([-bearing_outer_diameter/2-6,0,-1])rotate([0,-90,0])prismoid([11,10],[11,4], height=4, anchor=TOP+LEFT);

        // cutouts for reset buttons
        translate([-96.9, -35.1, -2])cyl(d=3.8, h=3, chamfer1=-0.6, anchor=BOTTOM);
        translate([-96.9, -35.1, -2])cuboid([10,2.4,3], chamfer=-0.6, edges=BOTTOM, anchor=BOTTOM+LEFT);
        translate([-96.9, -46.9, -2])cyl(d=3.8, h=3, chamfer1=-0.6, anchor=BOTTOM);
        translate([-96.9, -46.9, -2])cuboid([10,2.4,3], chamfer=-0.6, edges=BOTTOM, anchor=BOTTOM+LEFT);
        // text for reset and boot buttons
        translate([-87,-22,-2])linear_extrude(height = 1)rotate([180,0,-90])text("R", font="Arial:style=Bold", size=10);
        translate([-87,-50,-2])linear_extrude(height = 1)rotate([180,0,-90])text("B", font="Arial:style=Bold", size=10);
    }
    // bearing ring
    difference(){
        cyl(d=bearing_outer_diameter+10, h=9.7, rounding1=-1, rounding2=1, anchor=BOTTOM);
        translate([0,0,-3])cylinder(d=bearing_outer_diameter, h=15);
        // cutouts for clamps
        if(with_bearing_clips)
            for (w=[0:360/no_of_clips:360-360/no_of_clips])rotate([0,0,w])translate([-bearing_outer_diameter/2-2,-3.5, 1]) cube([4,7,10]);
        // cutout for hall sensor
        translate([-bearing_outer_diameter/2-6,0,0])rotate([0,-90,0])prismoid([11,10],[11,4], height=4, anchor=TOP+LEFT);
        translate([-bearing_outer_diameter/2-7,0,-1])cuboid([4, 5.65, 11], anchor=BOTTOM+LEFT);
    }
    // bearing clips
    if(with_bearing_clips){
        for (w=[0:360/no_of_clips:360-360/no_of_clips])rotate([0,0,180 + w - 180 * 5 / bearing_outer_diameter / PI])difference(){
            pie_slice(h = 10.1, d1=bearing_outer_diameter+3, d2=bearing_outer_diameter+1.4, ang = 360 * 5 / bearing_outer_diameter / PI, anchor=BOTTOM);
            cyl(h=10.1, d1=bearing_outer_diameter, d2=bearing_outer_diameter-2, rounding2=1, anchor=BOTTOM);
        }
    }
    // raiser ring for bearing
    difference(){
        cylinder(d=bearing_outer_diameter, h=0.7);
        translate([0,0,-1])cylinder(d=bearing_outer_diameter-5, h=2);
    }
    // walls
    difference(){
        hull(){
            translate([0,0,-1])cylinder(d=140, h=9.4);
            translate([-90,-60,-1])cylinder(d=20, h=9.4);
            translate([-90, 60,-1])cylinder(d=20, h=9.4);
        } 
        // inner cutout  
        hull(){
            translate([0,0,-2])cylinder(d=136, h=11);
            translate([-90,-60,-2])cylinder(d=16, h=11);
            translate([-90, 60,-2])cylinder(d=16, h=11);
        } 
        // cutout for rim
        hull(){
            translate([0,0,6])cylinder(d=138.2, h=3);
            translate([-90,-60,6])cylinder(d=18.2, h=3);
            translate([-90, 60,6])cylinder(d=18.2, h=3);
        } 
        // cutout for USB-C connector
        translate([-100,-41,1])cuboid([10,9.3,3.5], rounding=1.25, anchor=BOTTOM);
        // cutout for ESP Board
        translate([-98.5,-50,4])cube([2,18,5]);
        // make room for the reset buttons
        translate([-96.9, -35.1, -2])cyl(d=3.8, h=3, chamfer2=1, anchor=BOTTOM);
        translate([-96.9, -46.9, -2])cyl(d=3.8, h=3, chamfer2=1, anchor=BOTTOM);
    }

    // supports for ESP Board
    difference(){
        translate([-99,-49,0])cuboid([3,2,4.3], rounding=-1, edges=BOTTOM+RIGHT, anchor=BOTTOM+LEFT);
        translate([-96.9, -46.9, -2])cyl(d=3.8, h=3, chamfer2=1, anchor=BOTTOM);
    }
    difference(){
        translate([-99,-33,0])cuboid([3,2,4.3], rounding=-1, edges=BOTTOM+RIGHT, anchor=BOTTOM+LEFT);
        translate([-96.9, -35.1, -2])cyl(d=3.8, h=3, chamfer2=1, anchor=BOTTOM);
    }
    translate([-99,-51,0])cuboid([3,2,6],   rounding=-1, edges=BOTTOM, anchor=BOTTOM+LEFT);
    translate([-99,-31,0])cuboid([3,2,6],   rounding=-1, edges=BOTTOM, anchor=BOTTOM+LEFT);    
    
    translate([-79,-51,0])difference(){
        cuboid([4,6,5.5], rounding=-1, edges=BOTTOM, anchor=BOTTOM+LEFT);
        translate([0,1,4.3])cube([2,2,2]);
        translate([2,-0.5,0])cylinder(d=1.5, h=7, $fn=16);
    }
    translate([-79,-31,0])difference(){
        cuboid([4,6,5.5], rounding=-1, edges=BOTTOM, anchor=BOTTOM+LEFT);
        translate([0,-3,4.3])cube([2,2,2]);
        translate([2,0.5,0])cylinder(d=1.5, h=7, $fn=16);
    }
    
    // reset buttons for ESP
    translate([-96.9, -35.1, -1])cyl(d=1.6, h=4.4, rounding1=-0.7, rounding2=0.4, anchor=BOTTOM);
    translate([-96.9, -35.1, -2])cyl(d=3, h=1, chamfer1=0.4, anchor=BOTTOM);
    translate([-96.9, -35.1, -2])cuboid([15,1.8,1], chamfer=0.4, edges=BOTTOM, anchor=BOTTOM+LEFT);
    translate([-96.9, -46.9, -1])cyl(d=1.6, h=4.4, rounding1=-0.7, rounding2=0.4, anchor=BOTTOM);
    translate([-96.9, -46.9, -2])cyl(d=3, h=1, chamfer1=0.4, anchor=BOTTOM);
    translate([-96.9, -46.9, -2])cuboid([15,1.8,1], chamfer=0.4, edges=BOTTOM, anchor=BOTTOM+LEFT);

    // mounting for rotation sensor
    // clip
    translate([-motor_distance+23.6/2-1,0,0])cuboid([4,3,2], chamfer=1, edges=TOP, anchor=BOTTOM+LEFT);
    // post for clamp
    difference(){
        translate([-motor_distance-23.6/2,0,0])cuboid([4,8,4], chamfer=-1, edges=[BOTTOM+FRONT, BOTTOM+LEFT, BOTTOM+BACK], anchor=BOTTOM+RIGHT);
        translate([-motor_distance-23.6/2-2,0,0])cylinder(d=1.6, h=10);
    }

    // supports for TMC2209
    difference(){
        translate([-86,31.4,0])cuboid([4,20,4.8], chamfer=-1, edges=BOTTOM, anchor=BOTTOM+RIGHT);
        translate([-87,31.4,2.8])cuboid([4,15.7,5], anchor=BOTTOM+LEFT);
    }
    translate([-90,28.8,4.8])cuboid([4,3,2], chamfer=1, edges=TOP, anchor=BOTTOM+LEFT);

    difference(){
        translate([-67.3,31.4,0])cuboid([5,22,4.8], chamfer=-1, edges=BOTTOM, anchor=BOTTOM+LEFT);
        translate([-66.3,31.4,2.6])cuboid([4,15.7,5], anchor=BOTTOM+RIGHT);
        translate([-64.8,31.4+9,0])cylinder(d=1.6, h=10, $fn=16);
        translate([-64.8,31.4-9,0])cylinder(d=1.6, h=10, $fn=16);
       
    }

    // mounting posts
    translate([60, 0,0])mounting_post();
    translate([0, 60,0])mounting_post();
    translate([0,-60,0])mounting_post();
    translate([-90, 60,0])mounting_post();
    translate([-90,-60,0])mounting_post();

    //eventually print support ring
    if(with_supports){
       difference(){
            union(){
                translate([0,0,-2])cyl(d=extension_tube_outer_diameter-0.8, h=5.8, chamfer1=1, anchor=BOTTOM);
                for(w=[0:5:175])translate([0,0,3.6])rotate([0,0,w])cuboid([extension_tube_outer_diameter-0.2, 0.6, 0.4], chamfer=0.2, edges=[TOP+FRONT, TOP+BACK], anchor=BOTTOM);
            }
            translate([0,0,-2.1])cylinder(d=extension_tube_outer_diameter-5, h=7);
        }
    }
}

// top housing
module top() {
    difference(){
        hull(){
            translate([0,0,13.5])cyl(d=140, h=2,  chamfer2=1);
            translate([-90,-60,13.5])cyl(d=20, h=2, chamfer2=1);
            translate([-90, 60,13.5])cyl(d=20, h=2, chamfer2=1);
        }         
        //#translate([0,0,15.5-6])cyl(h=5, d2=96-6, d1=96, rounding2=2);
        // hole for tube
        cylinder(h=20, d=46);
        // mounting screws
        translate([60,   0,14.5])screw_hole("M3", length=10, head="flat large", thread=false, anchor=TOP);
        translate([0,  -60,14.5])screw_hole("M3", length=10, head="flat large", thread=false, anchor=TOP);
        translate([0,   60,14.5])screw_hole("M3", length=10, head="flat large", thread=false, anchor=TOP);
        translate([-90,-60,14.5])screw_hole("M3", length=10, head="flat large", thread=false, anchor=TOP);
        translate([-90, 60,14.5])screw_hole("M3", length=10, head="flat large", thread=false, anchor=TOP); 
        // hole for motor
        translate([-motor_distance, 0, 10])cylinder(d=16.5, h=10);
        // mounting holes for motor
        translate([-motor_distance,  43.85/2, 10])cylinder(d=3.3, h=10);
        translate([-motor_distance, -43.85/2, 10])cylinder(d=3.3, h=10);
        // holes for motor cover
        translate([ cover_width/2+3-motor_distance,  cover_length/2+3, 12.5])screw_hole("M3", length=10, head="flat large", thread=false, anchor=TOP, orient=DOWN);
        translate([ cover_width/2+3-motor_distance, -cover_length/2-3, 12.5])screw_hole("M3", length=10, head="flat large", thread=false, anchor=TOP, orient=DOWN);
        translate([-cover_width/2-3-motor_distance,  cover_length/2+3, 12.5])screw_hole("M3", length=10, head="flat large", thread=false, anchor=TOP, orient=DOWN);
        translate([-cover_width/2-3-motor_distance, -cover_length/2-3, 12.5])screw_hole("M3", length=10, head="flat large", thread=false, anchor=TOP, orient=DOWN);
        translate([-motor_distance-17.5,21-13,10])cube([6,13,10]);
    }
    
    // walls
    difference(){
        
        hull(){
            translate([0,0,6.5])cylinder(d=140, h=7);
            translate([-90,-60,6.5])cylinder(d=20, h=7);
            translate([-90, 60,6.5])cylinder(d=20, h=7);
        }   
        
        hull(){
            translate([0,0,2.5])cylinder(d=136, h=12);
            translate([-90,-60,2.5])cylinder(d=16, h=12);
            translate([-90, 60,2.5])cylinder(d=16, h=12);
        } 
        difference(){
            translate([-150,-150,5.4])cube([300,300,3]);
            hull(){
                translate([0,0,5])cylinder(d=137.8, h=4);
                translate([-90,-60,5])cylinder(d=17.8, h=4);
                translate([-90, 60,5])cylinder(d=17.8, h=4);
            }
        }
    }
}

module motor_cover(){

    difference(){
        hull(){
            translate([ cover_width/2,  cover_length/2, 0])cyl(d=20, h=15, chamfer2=1, anchor=BOTTOM);
            translate([ cover_width/2, -cover_length/2, 0])cyl(d=20, h=15, chamfer2=1, anchor=BOTTOM);
            translate([-cover_width/2,  cover_length/2, 0])cyl(d=20, h=15, chamfer2=1, anchor=BOTTOM);
            translate([-cover_width/2, -cover_length/2, 0])cyl(d=20, h=15, chamfer2=1, anchor=BOTTOM);
        }
        hull(){
            translate([ cover_width/2,  cover_length/2, -2])cyl(d=16, h=15, anchor=BOTTOM);
            translate([ cover_width/2, -cover_length/2, -2])cyl(d=16, h=15, anchor=BOTTOM);
            translate([-cover_width/2,  cover_length/2, -2])cyl(d=16, h=15, anchor=BOTTOM);
            translate([-cover_width/2, -cover_length/2, -2])cyl(d=16, h=15, anchor=BOTTOM);
        }
    }
    // posts for heat inserts
    translate([ cover_width/2+3,  cover_length/2+3, 0])difference(){ cyl(d=10, h=14, anchor=BOTTOM); cyl(d=4, h=13, chamfer1=-0.4, anchor=BOTTOM);}
    translate([ cover_width/2+3, -cover_length/2-3, 0])difference(){ cyl(d=10, h=14, anchor=BOTTOM); cyl(d=4, h=13, chamfer1=-0.4, anchor=BOTTOM);}
    translate([-cover_width/2-3,  cover_length/2+3, 0])difference(){ cyl(d=10, h=14, anchor=BOTTOM); cyl(d=4, h=13, chamfer1=-0.4, anchor=BOTTOM);}
    translate([-cover_width/2-3, -cover_length/2-3, 0])difference(){ cyl(d=10, h=14, anchor=BOTTOM); cyl(d=4, h=13, chamfer1=-0.4, anchor=BOTTOM);}
}

module assembly(){
    // ball bearing    
    translate([0,0,5.2])ball_bearing(id=bearing_inner_diameter,od=bearing_outer_diameter, width=bearing_height ,shield=false, $fn=128);

    // rotating ring
    color("blue")translate([0,0,1.9])rotating_ring();

    // rings
    color("black")translate([0,0,7.5])distance_ring(7);
    color("black")translate([0,0,-2 ])distance_ring(5);


    // motor assembly
    translate([-motor_distance,0,26.5])rotate([180,0,90]){
        color("red")import("Imports/Stepper.stl");
        color("green")translate([0,0,24.6])rotate([180,0,0])small_pulley();
    }

    // motor cover
    translate([-motor_distance,0,14.5])motor_cover();

    // rotary sensor
    translate([-motor_distance,0,-0.20-0.4]){
        difference(){
            color("white")cuboid([23.3,23.3,1.6], rounding=3.5, edges=[FWD+RIGHT,FWD+LEFT,BACK+RIGHT,BACK+LEFT]);
            translate([-8,-8,-2])cylinder(d=3.6, h=4, $fn=32);
            translate([-8, 8,-2])cylinder(d=3.6, h=4, $fn=32);
            translate([ 8,-8,-2])cylinder(d=3.6, h=4, $fn=32);
            translate([ 8, 8,-2])cylinder(d=3.6, h=4, $fn=32);
        }
        translate([-2.6,-2.6,0.8])color("black")cube([5.2,5.2,1.7]);
    }
    // rotary sensor clip
    translate([-motor_distance-23.6/2-2,0,8])rotate([180,0,0])color("black")sensor_clip();

    color("lightgrey")top();
    color("lightgrey")bottom();
        
    // camera - just for size
    //translate([0,0,16])color("lime")cylinder(d=80, h=75);

    // ESP and clip
    translate([-100+13.8,-50+2.9,5.3])rotate([-90,0,180])color("lime")import("Imports/XIAO ESP32S3.stl");
    translate([-77,-41,5.5])color("black")esp_clip();

    // TMC and clip
    translate([-68,25,3])rotate([0,180,-90])color("lime")import("Imports/TMC2209.stl");
    translate([-65.2,31.4,3.8])color("black")tmc_clip();

    // Hallsensor clip
    translate([-bearing_outer_diameter/2-2.1, 0, 2.7])rotate([0,-90,0])color("black")hall_clip();
}

assembly();
//small_pulley();
//rotating_ring();
//intersection(){
//    rotating_ring();
//    cylinder(d=50, h=12);
//    }

//motor_cover();
//assembly();
//difference(){ assembly();translate([-150,-200,-50])cube([300,200,100]);}
//top();
// intersection(){
//     bottom();
//     translate([0,0,-3])cylinder(d=52, h=20);
// }


//bottom();
//translate([-motor_distance-23.6/2-2,0,6])rotate([180,0,0])color("red")sensor_clip();
//hall_clip();

//translate([-bearing_outer_diameter/2-5+3-0.1,0,2.7])rotate([0,-90,0])color("black")hall_clip();
//translate([-68,25,3])rotate([0,180,-90])color("lime")import("Imports/TMC2209.stl");
//translate([-65.2,31.4,3.8])color("black")tmc_clip();

// intersection(){
//     bottom();
//     //translate([-76.35,31.4,0])cube(30, center=true);
//     translate([-101,-55,-3])cube([45,100,30]);
// }
//sensor_clip();
//tmc_clip();

//bottom();
//translate([0,0,1.9])rotating_ring();
//translate([0,0,1.9])rotating_ring();
/*
intersection() {
    bottom();
    translate([-101,-55,-3])cube([30,30,30]);
}
*/
    //translate([-100+13.8,-50+2.9,5.3])rotate([-90,0,180])color("lime")import("XIAO ESP32S3.stl");
    //translate([-77,-41,5.5])color("black")esp_clip();
//esp_clip();


/*
intersection(){
    bottom(); 
    translate([0,0,-5])cylinder(d=90, h=30);
}
*/

//translate([0,0,1.9])rotating_ring();
//translate([0,0,7.5])distance_ring(7);
//translate([0,0,5.2])ball_bearing(id=bearing_inner_diameter,od=bearing_outer_diameter, width=bearing_height ,shield=false, $fn=128);
//rotating_ring();


//small_pulley();
/*
difference(){
    cuboid([30,50,2], anchor=BOTTOM, chamfer=1, edges=TOP);
        // hole for motor
        translate([0, 0, -1])cylinder(d=16.2, h=10);
        // mounting holes for motor
        translate([0,  43.85/2, -1])cylinder(d=3.3, h=10);
        translate([0, -43.85/2, -1])cylinder(d=3.3, h=10);
        translate([ 10,  20, 2])screw_hole("M3", length=10, head="flat large", thread=false, anchor=TOP);
        translate([-10,  20, 2])screw_hole("M3", length=10, head="flat large", thread=false, anchor=TOP);
        translate([ 10, -20, 2])screw_hole("M3", length=10, head="flat large", thread=false, anchor=TOP);
        translate([-10, -20, 2])screw_hole("M3", length=10, head="flat large", thread=false, anchor=TOP);    
}

translate([40,0,0])union(){
    difference(){
        cuboid([30,50,2], anchor=BOTTOM, chamfer=1, edges=BOTTOM); 
        translate([0,0,1])cuboid([23.4,23.4,2], anchor=BOTTOM, rounding=3.5, edges=[FWD+RIGHT,FWD+LEFT,BACK+RIGHT,BACK+LEFT]);
    }
    
    translate([10,20,2])difference(){
        cyl(d=8, h=12.5, anchor=BOTTOM, rounding1=-1); 
        translate([0,0,6])cylinder(d=3.0, h=12);
    }
        translate([-10,20,2])difference(){
        cyl(d=8, h=12.5, anchor=BOTTOM, rounding1=-1); 
        translate([0,0,6])cylinder(d=3.0, h=12);
    }
        translate([10,-20,2])difference(){
        cyl(d=8, h=12.5, anchor=BOTTOM, rounding1=-1); 
        translate([0,0,6])cylinder(d=3.0, h=12);
    }
        translate([-10,-20,2])difference(){
        cyl(d=8, h=12.5, anchor=BOTTOM, rounding1=-1); 
        translate([0,0,6])cylinder(d=3.0, h=12);
    }
}
*/

/*
difference(){
    bottom();
    translate([-99.95,-49.95,-3])cube([24.5,21.3,3]);
    translate([-101,-39.5,-0.7])cuboid([14,9.3,3.5], rounding=1.25, anchor=BOTTOM);
}
translate([-100,-50,-2]) union(){
    difference(){
        rotate([-90,0,180])translate([-162.2555,-153.0555,-21.3360])import("seeed-top.stl");
        translate([0,14,0])cuboid([10,30,2], anchor=BOTTOM+RIGHT, edges=BOTTOM, chamfer=-1);
    }
    translate([13.8, 4.5, 5.6])rotate([-90,0,180])color("lime")import("XIAO ESP32S3.stl");
}
//import("seeed-bottom.3mf");
*/