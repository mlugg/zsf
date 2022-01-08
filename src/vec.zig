const std = @import("std");

pub const epsilon: f64 = 0.0001;
pub const Vec3 = std.meta.Vector(3, f64);

pub fn zero(v: Vec3) bool {
    return @reduce(.Add, @fabs(v)) < epsilon;
}

pub fn lengthSq(v: Vec3) f64 {
    return @reduce(.Add, v * v);
}

pub fn length(v: Vec3) f64 {
    return @sqrt(lengthSq(v));
}

pub fn dot(a: Vec3, b: Vec3) f64 {
    return @reduce(.Add, a * b);
}

pub fn cross(a: Vec3, b: Vec3) Vec3 {
    const M = std.meta.Vector(3, i32);
    return @shuffle(f64, a, b, M{ 1, 2, 0 }) * @shuffle(f64, a, b, M{ -3, -1, -2 }) -
        @shuffle(f64, a, b, M{ 2, 0, 1 }) * @shuffle(f64, a, b, M{ -2, -3, -1 });
}

pub fn normalize(v: Vec3) Vec3 {
    return v / @splat(3, length(v));
}

pub fn intersectLinePlane(line_dir: Vec3, line_point: Vec3, plane_norm: Vec3, plane_dist: f64) ?Vec3 {
    const t = dot(line_dir, plane_norm);
    if (t == 0) return null;
    const d = (plane_dist - dot(line_point, plane_norm)) / t;
    return line_point + @splat(3, d) * line_dir;
}
