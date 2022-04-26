const std = @import("std");
const zbsp = @import("zbsp.zig");
const parser = @import("parser.zig");

const Vec3 = std.meta.Vector(3, f64);

// BrushFinder - traverses the main BSP tree and finds every brush in it
// as well as the leaf containing it
const BrushFinder = struct {
    bsp: *zbsp.Bsp,
    result: std.AutoHashMap(*zbsp.Brush, *zbsp.Leaf),

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

    fn run(allocator: std.mem.Allocator, bsp: *zbsp.Bsp) !std.AutoHashMap(*zbsp.Brush, *zbsp.Leaf) {
        var self = BrushFinder{
            .bsp = bsp,
            .result = std.AutoHashMap(*zbsp.Brush, *zbsp.Leaf).init(allocator),
        };
        errdefer self.result.deinit();

        try self.checkNode(&self.bsp.nodes[0]);
        return self.result;
    }
};

// JoinFinder - finds any edges which lie on other brushes, i.e.
// potential seams
const Join = struct {
    brushes: [2]*zbsp.Brush,
    edge: zbsp.Edge,

    fn findForBrushList(allocator: std.mem.Allocator, bsp: *zbsp.Bsp, brushes: std.AutoHashMap(*zbsp.Brush, *zbsp.Leaf)) ![]Join {
        var result = std.ArrayList(Join).init(allocator);
        defer result.deinit();

        // This is dumb, but HashMap doesn't give us direct access to
        // the keys list
        var brush_list = std.ArrayList(*zbsp.Brush).init(allocator);
        defer brush_list.deinit();

        var it = brushes.keyIterator();
        while (it.next()) |brush| {
            try brush_list.append(brush.*);
        }

        for (brush_list.items) |brush, i| {
            for (brush_list.items[i + 1 ..]) |other| {
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

    var simple_complex: u32 = 0;
    var complex_complex: u32 = 0;

    for (joins) |j| {
        if (j.brushes[0].isBox(&bsp) != j.brushes[1].isBox(&bsp)) {
            // Simple-complex seam
            simple_complex += 1;
            try std.io.getStdOut().writer().print("sar_drawline {} {} {} {} {} {} 0 255 0\n", .{
                j.edge.points[0][0],
                j.edge.points[0][1],
                j.edge.points[0][2],
                j.edge.points[1][0],
                j.edge.points[1][1],
                j.edge.points[1][2],
            });
        }

        if (!j.brushes[0].isBox(&bsp) and !j.brushes[1].isBox(&bsp)) {
            // Complex-complex seam
            if (brush_leaves.get(j.brushes[0]) != brush_leaves.get(j.brushes[1])) {
                // Different visleaves!
                complex_complex += 1;
                try std.io.getStdOut().writer().print("sar_drawline {} {} {} {} {} {} 255 200 0\n", .{
                    j.edge.points[0][0],
                    j.edge.points[0][1],
                    j.edge.points[0][2],
                    j.edge.points[1][0],
                    j.edge.points[1][1],
                    j.edge.points[1][2],
                });
            }
        }
    }

    std.log.info("Found {} seamshots ({} simple-complex, {} complex-complex)", .{ simple_complex + complex_complex, simple_complex, complex_complex });
}
