upper_points = [
    [0, 43.52],
    [6.64, 43.52],
    [8.73, 43.15],
    [10.06, 42.07],
    [15.32, 33.11],
    [16.2, 31.29],
    [16.35, 29.56]];

lower_points = [
    [16.35, 29.56],
    [16.12, 24.25],
    [14.46, 17.78],
    [12.71, 12.18],
    [9.78, 7.09],
    [6.56, 2.34],
    [3.02, 0.16],
    [0,0]];

half_bottom_points = concat(
    lower_points,
    [
        [0, -5],
        [5, -5],
        [10, -2],
        [16, 6],
        [19, 13],
        [21, 19],
        [21, 29.56],
        ]);

half_top_points = concat(
    upper_points,
    [
        [16.35, 0],
        [50, 0],
        [50, 70],
        [0, 70]]);

module interfaceHole() {
    translate([0, 0, -1]) cylinder(h=14, r=3);
}

module interfaceHoles() {
    translate([10, 10, 0]) interfaceHole();
    translate([30, 10, 0]) interfaceHole();
    translate([10, 30, 0]) interfaceHole();
    translate([30, 30, 0]) interfaceHole();
}

module halfInterface() {
    difference () {
        cube(size = [40, 20, 5]);
        translate([10, 10, 0]) interfaceHole();
        translate([30, 10, 0]) interfaceHole();
    }
}

module interface() {
    union () {
        halfInterface();
        mirror([0, 1, 0]) halfInterface();
    }
}

module half_top_half () {
    linear_extrude(height=12) polygon(points=half_top_points);
}

module byBar () {
    color([1,0,0]) union() {
        half_top_half();
        mirror([1,0,0]) half_top_half();
    }
}

module holdBarNoHoles () {
    union () {
        byBar();
        translate([0,0,60]) byBar();
        translate([45, 70, 0]) rotate(a=90, v=[0,0,1]) cube(size=[12, 90, 72]);
    }
}

module hole () {
    cylinder(r=5, h=14);
}

module holes () {
    translate([30, 83, 36]) rotate(a=90, v=[1,0,0]) hole();
    translate([-30, 83, 36]) rotate(a=90, v=[1,0,0]) hole();
}

module holdBar () {
    difference () {
        holdBarNoHoles();
        holes();
        translate([-20, 82, 16]) rotate(a=90, v=[1,0,0]) interfaceHoles();
    }
}

holdBar();
