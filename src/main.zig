const std = @import("std");
const zbsp = @import("zbsp.zig");
const vec = @import("vec.zig");
const parser = @import("parser.zig");

const Vec3 = std.meta.Vector(3, f64);

// BrushFinder - traverses the main BSP tree and finds every brush in it
// as well as the leaf containing it
const BrushFinder = struct {
    bsp: *zbsp.Bsp,
    result: std.AutoArrayHashMap(*zbsp.Brush, *zbsp.Leaf),

    fn checkLeaf(self: *BrushFinder, leaf: *zbsp.Leaf) !void {
        for (leaf.leafbrushes(self.bsp)) |leafbrush| {
            const brush = leafbrush.get(self.bsp);

            if (brush.contents & 1 == 0) continue;
            if (self.result.contains(brush)) continue;
            try self.result.put(brush, leaf);
        }
    }

    fn checkNode(self: *BrushFinder, node: *zbsp.Node) anyerror!void {
        for (node.children(self.bsp)) |c| switch (c) {
            .node => |n| try self.checkNode(n),
            .leaf => |l| try self.checkLeaf(l),
        };
    }

    fn run(allocator: std.mem.Allocator, bsp: *zbsp.Bsp) !std.AutoArrayHashMap(*zbsp.Brush, *zbsp.Leaf) {
        var self = BrushFinder{
            .bsp = bsp,
            .result = std.AutoArrayHashMap(*zbsp.Brush, *zbsp.Leaf).init(allocator),
        };
        errdefer self.result.deinit();

        try self.checkNode(&self.bsp.nodes[0]);
        return self.result;
    }
};

const Join = struct {
    brushes: [2]*zbsp.Brush,
    edge: zbsp.Edge,

    // Finds any edges which lie on other brushes, i.e. potential seams
    fn findForBrushList(allocator: std.mem.Allocator, bsp: *zbsp.Bsp, brushes: std.AutoArrayHashMap(*zbsp.Brush, *zbsp.Leaf)) ![]Join {
        var result = std.ArrayList(Join).init(allocator);
        defer result.deinit();

        for (brushes.keys()) |brush| {
            for (brushes.keys()) |other| {
                for (try brush.edges(bsp)) |edge| {
                    if (other.fitEdgeToSurface(bsp, edge)) |adjusted| {
                        try result.append(.{ .brushes = .{ brush, other }, .edge = adjusted });
                    }
                }
            }
        }

        return result.toOwnedSlice();
    }
};

// SeamFinder - filters through through a list of joins for
// seams and merges coincident seams
const SeamFinder = struct {
    simp_comp: std.ArrayList(Join),
    comp_comp: std.ArrayList(Join),

    pub fn deinit(self: *SeamFinder) void {
        self.simp_comp.deinit();
        self.comp_comp.deinit();
    }

    fn addSeam(self: *SeamFinder, j: Join, comp: bool) !void {
        // early out - tiny seams can die
        if (vec.zero(j.edge.points[1] - j.edge.points[0])) return;

        const ptr = if (comp) &self.comp_comp else &self.simp_comp;

        // check for coincident seams
        for (ptr.items) |seam| {
            if (vec.zero(seam.edge.points[0] - j.edge.points[0]) and vec.zero(seam.edge.points[1] - j.edge.points[1])) {
                // identical seams
                return;
            }

            if (vec.zero(seam.edge.points[1] - j.edge.points[0]) and vec.zero(seam.edge.points[0] - j.edge.points[1])) {
                // identical seams, just vertices flipped
                return;
            }

            // TODO: merge partially-coincident seams
        }

        try ptr.append(j);
    }

    pub fn run(allocator: std.mem.Allocator, bsp: *zbsp.Bsp, joins: []Join, brush_leaves: std.AutoArrayHashMap(*zbsp.Brush, *zbsp.Leaf)) !SeamFinder {
        var self = SeamFinder{
            .simp_comp = std.ArrayList(Join).init(allocator),
            .comp_comp = std.ArrayList(Join).init(allocator),
        };

        for (joins) |j| {
            if (j.brushes[0].isBox(bsp) != j.brushes[1].isBox(bsp)) {
                try self.addSeam(j, false); // simple-complex
            } else if (!j.brushes[0].isBox(bsp) and !j.brushes[1].isBox(bsp)) {
                // check for different leaves
                if (brush_leaves.get(j.brushes[0]) != brush_leaves.get(j.brushes[1])) {
                    try self.addSeam(j, true); // complex-complex
                }
            }
        }

        return self;
    }
};

pub fn main() anyerror!void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer {
        _ = gpa.deinit();
    }

    const filename = blk: {
        var it = std.process.args();
        _ = it.skip();
        break :blk it.next() orelse return error.NoMapSpecified;
    };

    var f = try std.fs.cwd().openFile(filename, .{});
    defer f.close();

    var bsp = try parser.parse(gpa.allocator(), f);
    defer bsp.deinit();

    std.log.info(
        \\BSP parsed!
        \\  Planes: {}
        \\  Nodes: {}
        \\  Leaves: {}
        \\  Leafbrushes: {}
        \\  Brushes: {}
        \\  Brushsides: {}
    , .{
        bsp.planes.len,
        bsp.nodes.len,
        bsp.leaves.len,
        bsp.leafbrushes.len,
        bsp.brushes.len,
        bsp.brushsides.len,
    });

    var brush_leaves = try BrushFinder.run(gpa.allocator(), &bsp);
    defer brush_leaves.deinit();

    std.log.info("Successfully traversed tree - {} relevant brushes", .{brush_leaves.count()});

    const joins = try Join.findForBrushList(gpa.allocator(), &bsp, brush_leaves);
    defer gpa.allocator().free(joins);

    std.log.info("Found {} brush joins", .{joins.len});

    var sf = try SeamFinder.run(gpa.allocator(), &bsp, joins, brush_leaves);
    defer sf.deinit();

    const simp_comp = sf.simp_comp.items;
    const comp_comp = sf.comp_comp.items;

    for (simp_comp) |j| {
        try std.io.getStdOut().writer().print("sar_drawline {} {} {} {} {} {} 0 255 0\n", .{
            j.edge.points[0][0],
            j.edge.points[0][1],
            j.edge.points[0][2],
            j.edge.points[1][0],
            j.edge.points[1][1],
            j.edge.points[1][2],
        });
    }

    for (comp_comp) |j| {
        try std.io.getStdOut().writer().print("sar_drawline {} {} {} {} {} {} 255 200 0\n", .{
            j.edge.points[0][0],
            j.edge.points[0][1],
            j.edge.points[0][2],
            j.edge.points[1][0],
            j.edge.points[1][1],
            j.edge.points[1][2],
        });
    }

    std.log.info("Found {} seamshots ({} simple-complex, {} complex-complex)", .{ simp_comp.len + comp_comp.len, simp_comp.len, comp_comp.len });
}
