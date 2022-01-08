const std = @import("std");
const vec = @import("vec.zig");
const zbsp = @import("zbsp.zig");

pub fn parse(allocator: std.mem.Allocator, f: std.fs.File) !zbsp.Bsp {
    if (!try f.reader().isBytes(&.{ 'V', 'B', 'S', 'P' })) {
        return error.BadBspHeader;
    }

    const version = try f.reader().readIntLittle(i32);
    if (version != 21) {
        return error.UnknownBspVersion;
    }

    var lumps: [64]RawLump = undefined;
    for (lumps) |*l| {
        l.* = try RawLump.parse(f);
    }

    var arena = std.heap.ArenaAllocator.init(allocator);

    const planes = try parseLump(zbsp.Plane, parsePlane, arena.allocator(), f, lumps[1]);
    const nodes = try parseLump(zbsp.Node, parseNode, arena.allocator(), f, lumps[5]);
    const leaves = try parseLump(zbsp.Leaf, parseLeaf, arena.allocator(), f, lumps[10]);
    const leafbrushes = try parseLump(zbsp.LeafBrush, parseLeafBrush, arena.allocator(), f, lumps[17]);
    const brushes = try parseLump(zbsp.Brush, parseBrush, arena.allocator(), f, lumps[18]);
    const brushsides = try parseLump(zbsp.BrushSide, parseBrushSide, arena.allocator(), f, lumps[19]);

    return zbsp.Bsp{
        .planes = planes,
        .nodes = nodes,
        .leaves = leaves,
        .leafbrushes = leafbrushes,
        .brushes = brushes,
        .brushsides = brushsides,

        .arena = arena,
    };
}

fn parsePlane(r: anytype) !zbsp.Plane {
    const x = try r.readIntLittle(u32);
    const y = try r.readIntLittle(u32);
    const z = try r.readIntLittle(u32);
    const dist = try r.readIntLittle(u32);
    const type_ = try r.readIntLittle(i32);

    return zbsp.Plane{
        .norm = vec.Vec3{
            @bitCast(f32, x),
            @bitCast(f32, y),
            @bitCast(f32, z),
        },
        .dist = @bitCast(f32, dist),
        .type_ = type_,
    };
}

fn parseBrushSide(r: anytype) !zbsp.BrushSide {
    const plane_idx = try r.readIntLittle(u16);
    const texinfo_idx = try r.readIntLittle(i16);
    const dispinfo_idx = try r.readIntLittle(i16);
    const bevel = try r.readIntLittle(u16);

    if (texinfo_idx < 0) return error.BadBrushSide;
    if (dispinfo_idx < 0) return error.BadBrushSide;

    return zbsp.BrushSide{
        .plane_idx = plane_idx,
        .texinfo_idx = @intCast(u16, texinfo_idx),
        .dispinfo_idx = @intCast(u16, dispinfo_idx),
        .bevel = bevel == 1,
    };
}

fn parseBrush(r: anytype) !zbsp.Brush {
    const firstside = try r.readIntLittle(i32);
    const numsides = try r.readIntLittle(i32);
    const contents = try r.readIntLittle(i32);

    if (firstside < 0) return error.BadBrush;
    if (numsides < 0) return error.BadBrush;

    return zbsp.Brush{
        .firstside = @intCast(u32, firstside),
        .numsides = @intCast(u32, numsides),
        .contents = contents,
    };
}

fn parseNode(r: anytype) !zbsp.Node {
    const planenum = try r.readIntLittle(i32);
    const children = [_]i32{
        try r.readIntLittle(i32),
        try r.readIntLittle(i32),
    };
    const mins = [_]i16{
        try r.readIntLittle(i16),
        try r.readIntLittle(i16),
        try r.readIntLittle(i16),
    };
    const maxs = [_]i16{
        try r.readIntLittle(i16),
        try r.readIntLittle(i16),
        try r.readIntLittle(i16),
    };
    const firstface = try r.readIntLittle(u16);
    const numfaces = try r.readIntLittle(u16);
    const area = try r.readIntLittle(i16);
    try r.skipBytes(2, .{});

    if (planenum < 0) return error.BadNode;

    return zbsp.Node{
        .planenum = @intCast(u32, planenum),
        .children_ = children,
        .mins = mins,
        .maxs = maxs,
        .firstface = firstface,
        .numfaces = numfaces,
        .area = area,
    };
}

fn parseLeaf(r: anytype) !zbsp.Leaf {
    const contents = try r.readIntLittle(i32);
    const cluster = try r.readIntLittle(u16);
    const area_flags = try r.readIntLittle(u16);
    const area = @truncate(u9, area_flags);
    const flags = @intCast(u7, area_flags >> 9);
    const mins = [_]i16{
        try r.readIntLittle(i16),
        try r.readIntLittle(i16),
        try r.readIntLittle(i16),
    };
    const maxs = [_]i16{
        try r.readIntLittle(i16),
        try r.readIntLittle(i16),
        try r.readIntLittle(i16),
    };
    const firstleafface = try r.readIntLittle(u16);
    const numleaffaces = try r.readIntLittle(u16);
    const firstleafbrush = try r.readIntLittle(u16);
    const numleafbrushes = try r.readIntLittle(u16);
    const leaf_water_data_id = try r.readIntLittle(i16);

    try r.skipBytes(2, .{});

    return zbsp.Leaf{
        .contents = contents,
        .cluster = cluster,
        .area = area,
        .flags = flags,
        .mins = mins,
        .maxs = maxs,
        .firstleafface = firstleafface,
        .numleaffaces = numleaffaces,
        .firstleafbrush = firstleafbrush,
        .numleafbrushes = numleafbrushes,
        .leaf_water_data_id = leaf_water_data_id,
    };
}

fn parseLeafBrush(r: anytype) !zbsp.LeafBrush {
    return zbsp.LeafBrush{
        .idx = try r.readIntLittle(u16),
    };
}

fn parseLump(comptime T: type, comptime parseFn: anytype, allocator: std.mem.Allocator, f: std.fs.File, raw: RawLump) ![]T {
    try f.seekTo(raw.offset);
    var r = std.io.limitedReader(f.reader(), raw.len).reader();

    var arr = std.ArrayList(T).init(allocator);
    defer arr.deinit();

    while (true) {
        const elem = parseFn(r) catch |err| switch (err) {
            error.EndOfStream => break,
            else => |e| return e,
        };
        try arr.append(elem);
    }

    return arr.toOwnedSlice();
}

const RawLump = struct {
    offset: u32,
    len: u32,
    version: u32,

    fn parse(f: std.fs.File) !RawLump {
        const offset = try f.reader().readIntLittle(i32);
        const len = try f.reader().readIntLittle(i32);
        const version = try f.reader().readIntLittle(i32);

        var ident: [4]u8 = undefined;
        try f.reader().readNoEof(&ident);

        if (offset < 0) return error.BadBspLump;
        if (len < 0) return error.BadBspLump;
        if (version < 0) return error.BadBspLump;

        if (!std.mem.eql(u8, &ident, &.{ 0, 0, 0, 0 })) {
            // TODO
            return error.CompressedBspLump;
        }

        return RawLump{
            .offset = @intCast(u32, offset),
            .len = @intCast(u32, len),
            .version = @intCast(u32, version),
        };
    }
};
