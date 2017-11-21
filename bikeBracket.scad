upper_points = [
    [0, 43.52],
    [6.64, 43.52],
    [8.73, 43.15],
    [10.06, 42.07],
    [15.32, 33.11],
    [16.2, 31.29],
    [16.35, 29.56]];

lower_points = [
    [16.12, 24.25],
    [14.46, 17.78],
    [12.71, 12.18],
    [9.78, 7.09],
    [6.56, 2.34],
    [3.02, 0.16],
    [0,0]];

half_top_points = concat(
    upper_points,
    [
        [21, 29.56],
        [21, 34.56],
        [13, 49],
        [3.5, 49],
        [3.5, 69],
        [0, 69]]);

module topHole(x, z) {
    translate([x, 76, z]) rotate(a = 90, v = [1, 0, 0]) linear_extrude(height=8) circle(4);
}

module interfaceHole() {
    translate([0, 0, -1]) cylinder(h=9, r=3);
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
    union () {
        linear_extrude(height=40) polygon(points=half_top_points);
        {
            translate([20, 34.56, 40])
            rotate(a=90, v=[0,1,0])
            rotate(a=90, v=[1,0,0])
            halfInterface();
        }
    }
}
