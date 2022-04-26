const std = @import("std");
const vec = @import("vec.zig");

const Vec3 = vec.Vec3;

const epsilon = 0.0001;

fn intersectPlanes(p0: Plane, p1: Plane, p2: Plane) ?Vec3 {
    var det = vec.dot(vec.cross(p0.norm, p1.norm), p2.norm);
    if (@fabs(det) < epsilon) return null;

    const v = @splat(3, p0.dist) * vec.cross(p1.norm, p2.norm) + @splat(3, p1.dist) * vec.cross(p2.norm, p0.norm) + @splat(3, p2.dist) * vec.cross(p0.norm, p1.norm);

    return v / @splat(3, det);
}

pub const Bsp = struct {
    planes: []Plane,
    nodes: []Node,
    leaves: []Leaf,
    leafbrushes: []LeafBrush,
    brushes: []Brush,
    brushsides: []BrushSide,

    arena: std.heap.ArenaAllocator,

    pub fn deinit(self: *Bsp) void {
        self.arena.deinit();
    }
};

pub const Plane = struct {
    norm: Vec3,
    dist: f64,
    type_: i32,
};

pub const BrushSide = struct {
    plane_idx: u16,
    texinfo_idx: u16,
    dispinfo_idx: u16,
    bevel: bool,

    pub fn plane(self: BrushSide, bsp: *Bsp) *Plane {
        return &bsp.planes[self.plane_idx];
    }
};

pub const Edge = struct {
    points: [2]Vec3,
};

pub const Brush = struct {
    firstside: u32,
    numsides: u32,
    contents: i32,
    _edges: ?[]Edge = null,

    pub fn sides(self: Brush, bsp: *Bsp) []BrushSide {
        return bsp.brushsides[self.firstside .. self.firstside + self.numsides];
    }

    pub fn isBox(self: Brush, bsp: *Bsp) bool {
        if (self.sides(bsp).len != 6) return false;

        for (self.sides(bsp)) |s| {
            if (s.plane(bsp).type_ > 2) return false;
        }

        return true;
    }

    pub fn edges(self: *Brush, bsp: *Bsp) ![]Edge {
        if (self._edges) |edge_arr| return edge_arr;

        const allocator = bsp.arena.allocator();

        var edge_al = std.ArrayList(Edge).init(allocator);
        defer edge_al.deinit();

        for (self.sides(bsp)) |a, i| {
            if (a.bevel) continue;
            for (self.sides(bsp)[i + 1 ..]) |b, j| {
                if (b.bevel) continue;

                const pa = a.plane(bsp);
                const pb = b.plane(bsp);

                if (vec.zero(pa.norm - pb.norm)) continue;
                if (vec.zero(pa.norm + pb.norm)) continue;

                const line_dir = vec.normalize(vec.cross(pa.norm, pb.norm));

                var line_point: Vec3 = undefined;

                if (line_dir[0] != 0) {
                    line_point[0] = 0;
                    line_point[2] = (pa.norm[1] * pb.dist - pb.norm[1] * pa.dist) / (pa.norm[1] * pb.norm[2] - pa.norm[2] * pb.norm[1]);
                    line_point[1] = if (pa.norm[1] != 0)
                        (pa.dist - pa.norm[2] * line_point[2]) / pa.norm[1]
                    else
                        (pb.dist - pb.norm[2] * line_point[2]) / pb.norm[1];
                } else if (line_dir[1] != 0) {
                    line_point[1] = 0;
                    line_point[2] = (pa.norm[0] * pb.dist - pb.norm[0] * pa.dist) / (pa.norm[0] * pb.norm[2] - pa.norm[2] * pb.norm[0]);
                    line_point[0] = if (pa.norm[0] != 0)
                        (pa.dist - pa.norm[2] * line_point[2]) / pa.norm[0]
                    else
                        (pb.dist - pb.norm[2] * line_point[2]) / pb.norm[0];
                } else {
                    line_point[2] = 0;
                    line_point[1] = (pa.norm[0] * pb.dist - pb.norm[0] * pa.dist) / (pa.norm[0] * pb.norm[1] - pa.norm[1] * pb.norm[0]);
                    line_point[0] = if (pa.norm[0] != 0)
                        (pa.dist - pa.norm[1] * line_point[1]) / pa.norm[0]
                    else
                        (pb.dist - pb.norm[1] * line_point[1]) / pb.norm[0];
                }

                if (std.math.isNan(line_point[0])) continue;
                if (std.math.isNan(line_point[1])) continue;
                if (std.math.isNan(line_point[2])) continue;

                var min_dist = -std.math.inf_f64;
                var max_dist = std.math.inf_f64;

                for (self.sides(bsp)) |c, k| {
                    if (i == k or j + i + 1 == k) continue;
                    if (c.bevel) continue;

                    const pc = c.plane(bsp);

                    if (vec.dot(pc.norm, line_dir) == 0) {
                        if (vec.dot(pc.norm, line_point) - pc.dist > epsilon) {
                            min_dist = -std.math.inf_f64;
                            max_dist = -std.math.inf_f64;
                            break;
                        }
                        continue;
                    }

                    const t = (pc.dist - vec.dot(pc.norm, line_point)) / vec.dot(pc.norm, line_dir);
                    if (t < max_dist and vec.dot(pc.norm, line_dir) > 0) {
                        max_dist = t;
                    } else if (t > min_dist and vec.dot(pc.norm, line_dir) < 0) {
                        min_dist = t;
                    }
                }

                if (max_dist > min_dist) {
                    const p1 = line_point + line_dir * @splat(3, min_dist);
                    const p2 = line_point + line_dir * @splat(3, max_dist);

                    try edge_al.append(.{ .points = .{ p1, p2 } });
                }
            }
        }

        const edge_arr = edge_al.toOwnedSlice();
        self._edges = edge_arr;
        return edge_arr;
    }

    pub fn fitEdgeToSurface(self: Brush, bsp: *Bsp, edge: Edge) ?Edge {
        var adjusted = edge;

        const line_dir = vec.normalize(edge.points[1] - edge.points[0]);
        const line_point = edge.points[0];

        for (self.sides(bsp)) |side| {
            const p = side.plane(bsp);
            const dist0 = vec.dot(adjusted.points[0], p.norm) - p.dist;
            const dist1 = vec.dot(adjusted.points[1], p.norm) - p.dist;

            if (dist0 > epsilon and dist1 > epsilon) return null; // Outside brush

            if (dist0 > 0) {
                // Clip first point to be on plane
                // Find intersection of the edge line with the plane
                if (vec.intersectLinePlane(line_dir, line_point, p.norm, p.dist)) |point| {
                    // Successful intersection - move the point
                    adjusted.points[0] = point;
                } else {
                    // The line is parallel to this plane. We'll let
                    // the case slide only if the distance is less than
                    // epsilon, otherwise it's far enough out that we
                    // assume it's not on the brush
                    if (dist0 > epsilon) return null;
                }
            }

            if (dist1 > 0) {
                // Clip second point to be on plane
                // Find intersection of the edge line with the plane
                if (vec.intersectLinePlane(line_dir, line_point, p.norm, p.dist)) |point| {
                    // Successful intersection - move the point
                    adjusted.points[1] = point;
                } else {
                    // The line is parallel to this plane. We'll let
                    // the case slide only if the distance is less than
                    // epsilon, otherwise it's far enough out that we
                    // assume it's not on the brush
                    if (dist1 > epsilon) return null;
                }
            }

            // If the endpoints are in the same place, then there's
            // nothing here
            if (vec.zero(adjusted.points[0] - adjusted.points[1])) return null;
        }

        return adjusted;
    }
};

pub const LeafBrush = struct {
    idx: u16,

    pub fn get(self: LeafBrush, bsp: *Bsp) *Brush {
        return &bsp.brushes[self.idx];
    }
};

pub const Node = struct {
    planenum: u32,
    children_: [2]i32,
    mins: [3]i16,
    maxs: [3]i16,
    firstface: u16,
    numfaces: u16,
    area: i16,

    const Child = union(enum) {
        node: *Node,
        leaf: *Leaf,
    };

    pub fn plane(self: Node, bsp: *Bsp) *Plane {
        return &bsp.planes[self.planenum];
    }

    pub fn children(self: Node, bsp: *Bsp) [2]Child {
        const idx0 = self.children_[0];
        const idx1 = self.children_[1];

        return .{
            if (idx0 < 0) .{ .leaf = &bsp.leaves[@intCast(u32, -idx0 - 1)] } else .{ .node = &bsp.nodes[@intCast(u32, idx0)] },
            if (idx1 < 0) .{ .leaf = &bsp.leaves[@intCast(u32, -idx1 - 1)] } else .{ .node = &bsp.nodes[@intCast(u32, idx1)] },
        };
    }
};

pub const Leaf = struct {
    contents: i32,
    cluster: u16,
    area: u9,
    flags: u7,
    mins: [3]i16,
    maxs: [3]i16,
    firstleafface: u16,
    numleaffaces: u16,
    firstleafbrush: u16,
    numleafbrushes: u16,
    leaf_water_data_id: i16,

    pub fn leafbrushes(self: Leaf, bsp: *Bsp) []LeafBrush {
        return bsp.leafbrushes[self.firstleafbrush .. self.firstleafbrush + self.numleafbrushes];
    }
};
