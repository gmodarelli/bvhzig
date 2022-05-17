const std = @import("std");
const ztracy = @import("ztracy");
const endianness = @import("builtin").target.cpu.arch.endian();
const c = @cImport({
    @cInclude("SDL.h");
});

const math = std.math;

fn print(comptime format: []const u8, args: anytype) void {
    std.debug.getStderrMutex().lock();
    defer std.debug.getStderrMutex().unlock();
    const stderr = std.io.getStdErr().writer();
    nosuspend stderr.print(format ++ "\n", args) catch return;
}

fn float3Min(a: [3]f32, b: [3]f32) [3]f32 {
    return .{ math.min(a[0], b[0]), math.min(a[1], b[1]), math.min(a[2], b[2]) };
}

fn float3Max(a: [3]f32, b: [3]f32) [3]f32 {
    return .{ math.max(a[0], b[0]), math.max(a[1], b[1]), math.max(a[2], b[2]) };
}

fn float3Sub(a: [3]f32, b: [3]f32) [3]f32 {
    return .{ a[0] - b[0], a[1] - b[1], a[2] - b[2] };
}

fn float3Add(a: [3]f32, b: [3]f32) [3]f32 {
    return .{ a[0] + b[0], a[1] + b[1], a[2] + b[2] };
}

fn float3MulScalar(a: [3]f32, b: f32) [3]f32 {
    return .{ a[0] * b, a[1] * b, a[2] * b };
}

fn float3Cross(a: [3]f32, b: [3]f32) [3]f32 {
    return .{ a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] };
}

fn float3Dot(a: [3]f32, b: [3]f32) f32 {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

fn float3Normalize(v: [3]f32) [3]f32 {
    const inv_length: f32 = 1.0 / math.sqrt(float3Dot(v, v));
    return float3MulScalar(v, inv_length);
}

const Tri = struct {
    vertex0: [3]f32,
    vertex1: [3]f32,
    vertex2: [3]f32,
    centroid: [3]f32,
};

const BVHNode = struct {
    aabb_min: [3]f32,
    aabb_max: [3]f32,
    left_first: u32,
    tri_count: u32,

    pub fn is_leaf(node: *BVHNode) bool {
        return node.tri_count > 0;
    }

    pub fn calculateNodeCost(node: *BVHNode) f32 {
        const e = float3Sub(node.aabb_max, node.aabb_min);
        const area = e[0] * e[1] + e[1] * e[2] + e[2] * e[0];
        return @intToFloat(f32, node.tri_count) * area;
    }
};

const Ray = struct {
    origin: [3]f32,
    direction: [3]f32,
    r_direction: [3]f32,
    t: f32 = 1.0e+30,
};

const AABB = struct {
    min: [3]f32,
    max: [3]f32,

    pub fn init() AABB {
        return .{
            .min = .{ 1.0e+30, 1.0e+30, 1.0e+30 },
            .max = .{ -1.0e+30, -1.0e+30, -1.0e+30 },
        };
    }

    pub fn grow(aabb: *AABB, p: [3]f32) void {
        aabb.min = float3Min(aabb.min, p);
        aabb.max = float3Max(aabb.max, p);
    }

    pub fn growWithAABB(aabb: *AABB, other: *AABB) void {
        if (other.min[0] != 1.0e+30) {
            aabb.grow(other.min);
            aabb.grow(other.max);
        }
    }

    pub fn area(aabb: *AABB) f32 {
        const e: [3]f32 = float3Sub(aabb.max, aabb.min);
        return e[0] * e[1] + e[1] * e[2] + e[2] * e[0];
    }
};

const Bin = struct {
    bounds: AABB,
    tri_count: u32,
};

const num_bins: u32 = 8;

pub fn intersectTri(ray: *Ray, tri: *const Tri) void {
    const edge1 = float3Sub(tri.vertex1, tri.vertex0);
    const edge2 = float3Sub(tri.vertex2, tri.vertex0);
    const h = float3Cross(ray.direction, edge2);
    const a = float3Dot(edge1, h);
    if (a > -0.0001 and a < 0.0001) { // Ray parallel to triangle
        return;
    }

    const f: f32 = 1.0 / a;
    const s = float3Sub(ray.origin, tri.vertex0);
    const u = f * float3Dot(s, h);
    if (u < 0 or u > 1) {
        return;
    }

    const q = float3Cross(s, edge1);
    const v = f * float3Dot(ray.direction, q);
    if (v < 0 or u + v > 1) {
        return;
    }

    const t = f * float3Dot(edge2, q);
    if (t > 0.0001) {
        ray.t = math.min(ray.t, t);
    }
}

pub fn intersectAABB(ray: *Ray, bmin: [3]f32, bmax: [3]f32) f32 {
    var tx1: f32 = (bmin[0] - ray.origin[0]) * ray.r_direction[0];
    var tx2: f32 = (bmax[0] - ray.origin[0]) * ray.r_direction[0];
    var tmin: f32 = math.min(tx1, tx2);
    var tmax: f32 = math.max(tx1, tx2);

    var ty1: f32 = (bmin[1] - ray.origin[1]) * ray.r_direction[1];
    var ty2: f32 = (bmax[1] - ray.origin[1]) * ray.r_direction[1];
    tmin = math.max(tmin, math.min(ty1, ty2));
    tmax = math.min(tmax, math.max(ty1, ty2));

    var tz1: f32 = (bmin[2] - ray.origin[2]) * ray.r_direction[2];
    var tz2: f32 = (bmax[2] - ray.origin[2]) * ray.r_direction[2];
    tmin = math.max(tmin, math.min(tz1, tz2));
    tmax = math.min(tmax, math.max(tz1, tz2));

    if (tmax >= tmin and tmin < ray.t and tmax > 0) {
        return tmin;
    }

    return 1.0e+30;
}

const BasicBVHApp = struct {
    num_triangles: u32 = 0,
    triangles: []Tri,
    original: []Tri,
    triangle_indices: []u32,
    bvh_nodes: []BVHNode,
    root_node_index: u32 = 0,
    node_used: u32 = 1,
    r: f32 = 0,

    pub fn init(allocator: std.mem.Allocator) BasicBVHApp {
        var app: BasicBVHApp = undefined;
        app.root_node_index = 0;
        app.node_used = 1;

        app.loadTrianglesFromMesh(allocator) catch unreachable;

        std.debug.assert(app.num_triangles > 0);
        app.bvh_nodes = allocator.alloc(BVHNode, app.num_triangles * 2) catch unreachable;

        const start = std.time.milliTimestamp();
        app.animate();
        app.buildBVH();
        const end = std.time.milliTimestamp();
        const elapsed: f32 = @intToFloat(f32, end - start);
        print("BVH build time: {d:.2}ms", .{elapsed});

        return app;
    }

    pub fn deinit(app: *BasicBVHApp, allocator: std.mem.Allocator) void {
        allocator.free(app.triangles);
        allocator.free(app.original);
        allocator.free(app.triangle_indices);
        allocator.free(app.bvh_nodes);
    }

    fn loadTrianglesFromMesh(app: *BasicBVHApp, allocator: std.mem.Allocator) !void {
        const start = std.time.milliTimestamp();

        const dir = std.fs.cwd();
        var file = try dir.openFile("content/bigben.tri", .{ .mode = .read_only });

        var reader = file.reader();
        var buffer: [512]u8 = undefined;
        app.num_triangles = 0;
        while (try reader.readUntilDelimiterOrEof(buffer[0..], '\n')) |_| {
            app.num_triangles += 1;
        }
        file.close();

        app.triangles = allocator.alloc(Tri, app.num_triangles) catch unreachable;
        app.original = allocator.alloc(Tri, app.num_triangles) catch unreachable;
        app.triangle_indices = allocator.alloc(u32, app.num_triangles) catch unreachable;

        // TODO: Figure out a way to rewind the file reader instead of re-opening the file again
        file = try dir.openFile("content/bigben.tri", .{ .mode = .read_only });
        defer file.close();
        reader = file.reader();
        var i: u32 = 0;
        while (try reader.readUntilDelimiterOrEof(buffer[0..], '\n')) |line| {
            var it = std.mem.tokenize(u8, line, " ");

            var triangle: Tri = undefined;
            triangle.vertex0[0] = try std.fmt.parseFloat(f32, it.next().?);
            triangle.vertex0[1] = try std.fmt.parseFloat(f32, it.next().?);
            triangle.vertex0[2] = try std.fmt.parseFloat(f32, it.next().?);

            triangle.vertex1[0] = try std.fmt.parseFloat(f32, it.next().?);
            triangle.vertex1[1] = try std.fmt.parseFloat(f32, it.next().?);
            triangle.vertex1[2] = try std.fmt.parseFloat(f32, it.next().?);

            triangle.vertex2[0] = try std.fmt.parseFloat(f32, it.next().?);
            triangle.vertex2[1] = try std.fmt.parseFloat(f32, it.next().?);
            triangle.vertex2[2] = try std.fmt.parseFloat(f32, it.next().?);

            app.original[i] = triangle;
            i += 1;
        }
        const end = std.time.milliTimestamp();
        const elapsed: f32 = @intToFloat(f32, end - start);
        print("Mesh loaded in {d:.2}ms", .{elapsed});
    }

    pub fn animate(bvh: *BasicBVHApp) void {
        bvh.r += 0.05;
        if (bvh.r > 2.0 * math.pi) {
            bvh.r = bvh.r - 2 * math.pi;
        }

        const a: f32 = @sin(bvh.r) * 0.5;
        var i: u32 = 0;
        while (i < bvh.num_triangles) : (i += 1) {
            var v = bvh.original[i].vertex0;
            var s: f32 = a * (v[1] - 0.2) * 0.2;
            var x: f32 = v[0] * @cos(s) - v[1] * @sin(s);
            var y: f32 = v[0] * @sin(s) + v[1] * @cos(s);
            bvh.triangles[i].vertex0 = .{ x, y, v[2] };

            v = bvh.original[i].vertex1;
            s = a * (v[1] - 0.2) * 0.2;
            x = v[0] * @cos(s) - v[1] * @sin(s);
            y = v[0] * @sin(s) + v[1] * @cos(s);
            bvh.triangles[i].vertex1 = .{ x, y, v[2] };

            v = bvh.original[i].vertex2;
            s = a * (v[1] - 0.2) * 0.2;
            x = v[0] * @cos(s) - v[1] * @sin(s);
            y = v[0] * @sin(s) + v[1] * @cos(s);
            bvh.triangles[i].vertex2 = .{ x, y, v[2] };
        }
    }

    pub fn intersectBVH(bvh: *BasicBVHApp, ray: *Ray, node_index: u32) void {
        var node = &bvh.bvh_nodes[node_index];
        var stack: [64]*BVHNode = undefined;

        var stack_ptr: u32 = 0;
        while (true) {
            if (node.is_leaf()) {
                var i: u32 = 0;
                while (i < node.tri_count) : (i += 1) {
                    intersectTri(ray, &bvh.triangles[bvh.triangle_indices[node.left_first + i]]);
                }

                if (stack_ptr == 0) {
                    break;
                } else {
                    stack_ptr -= 1;
                    node = stack[stack_ptr];
                    continue;
                }
            }

            var child1 = &bvh.bvh_nodes[node.left_first];
            var child2 = &bvh.bvh_nodes[node.left_first + 1];
            var dist1 = intersectAABB(ray, child1.aabb_min, child1.aabb_max);
            var dist2 = intersectAABB(ray, child2.aabb_min, child2.aabb_max);
            if (dist1 > dist2) {
                var tmp_dist = dist1;
                dist1 = dist2;
                dist2 = tmp_dist;

                var tmp_child = child1;
                child1 = child2;
                child2 = tmp_child;
            }

            if (dist1 == 1.0e+30) {
                if (stack_ptr == 0) {
                    break;
                } else {
                    stack_ptr -= 1;
                    node = stack[stack_ptr];
                }
            } else {
                node = child1;
                if (dist2 != 1.0e+30) {
                    stack[stack_ptr] = child2;
                    stack_ptr += 1;
                }
            }
        }
    }

    fn buildBVH(app: *BasicBVHApp) void {
        app.node_used = 1;

        // Populate triangle index array
        var i: u32 = 0;
        while (i < app.num_triangles) : (i += 1) {
            app.triangle_indices[i] = i;
        }

        // Calculate triangle centroids for partitioning
        i = 0;
        while (i < app.num_triangles) : (i += 1) {
            app.triangles[i].centroid = .{
                (app.triangles[i].vertex0[0] + app.triangles[i].vertex1[0] + app.triangles[i].vertex2[0]) * 0.3333,
                (app.triangles[i].vertex0[1] + app.triangles[i].vertex1[1] + app.triangles[i].vertex2[1]) * 0.3333,
                (app.triangles[i].vertex0[2] + app.triangles[i].vertex1[2] + app.triangles[i].vertex2[2]) * 0.3333,
            };
        }

        // Assign all triangles to the root node
        var bvh_node = &app.bvh_nodes[app.root_node_index];
        bvh_node.left_first = 0;
        bvh_node.tri_count = app.num_triangles;

        app.updateNodeBounds(app.root_node_index);

        // Subdivide recursively
        app.subdivide(app.root_node_index);
    }

    fn refitBVH(app: *BasicBVHApp) void {
        var i: i32 = @intCast(i32, app.node_used - 1);
        while (i >= 0) : (i -= 1) {
            var node = &app.bvh_nodes[@intCast(u32, i)];
            if (node.is_leaf()) {
                // Leaf node: adjust bounds to contained triangles
                app.updateNodeBounds(@intCast(u32, i));
                continue;
            }

            // interior node: adjust bounds to child node bounds
            var left_child = &app.bvh_nodes[node.left_first];
            var right_child = &app.bvh_nodes[node.left_first + 1];

            node.aabb_min = float3Min(left_child.aabb_min, right_child.aabb_min);
            node.aabb_max = float3Max(left_child.aabb_max, right_child.aabb_max);
        }
    }

    fn updateNodeBounds(app: *BasicBVHApp, node_index: u32) void {
        std.debug.assert(node_index < app.node_used);

        var node = &app.bvh_nodes[node_index];
        node.aabb_min = .{ 1.0e+30, 1.0e+30, 1.0e+30 };
        node.aabb_max = .{ -1.0e+30, -1.0e+30, -1.0e+30 };

        var i: u32 = 0;
        const first: u32 = node.left_first;

        while (i < node.tri_count) : (i += 1) {
            var left_tri_idx = app.triangle_indices[first + i];
            const left_tri = app.triangles[left_tri_idx];

            node.aabb_min = float3Min(node.aabb_min, left_tri.vertex0);
            node.aabb_min = float3Min(node.aabb_min, left_tri.vertex1);
            node.aabb_min = float3Min(node.aabb_min, left_tri.vertex2);
            node.aabb_max = float3Max(node.aabb_max, left_tri.vertex0);
            node.aabb_max = float3Max(node.aabb_max, left_tri.vertex1);
            node.aabb_max = float3Max(node.aabb_max, left_tri.vertex2);
        }
    }

    fn subdivide(app: *BasicBVHApp, node_index: u32) void {
        std.debug.assert(node_index < app.node_used);
        var node = &app.bvh_nodes[node_index];

        // Determine split axis using SAH
        var axis: u32 = 0;
        var split_pos: f32 = 0;
        var split_cost = app.findBestSplitPlane(node, &axis, &split_pos);

        // Calculate parent cost to terminate recursion
        const nosplit_cost = node.calculateNodeCost();

        if (split_cost >= nosplit_cost) {
            return;
        }

        std.debug.assert(axis >= 0 and axis < 3);
        std.debug.assert(split_cost < 1.0e+30);

        // In-place partition
        var i: u32 = node.left_first;
        var j: u32 = i + node.tri_count - 1;
        while (i <= j) {
            if (app.triangles[app.triangle_indices[i]].centroid[axis] < split_pos) {
                i += 1;
            } else {
                var tmp = app.triangle_indices[i];
                app.triangle_indices[i] = app.triangle_indices[j];
                app.triangle_indices[j] = tmp;
                j -= 1;
            }
        }

        // Abort split if one of the sides is empty
        const left_count: u32 = i - node.left_first;
        if (left_count == 0 or left_count == node.tri_count) {
            return;
        }

        // Create child nodes
        const left_child_idx = app.node_used;
        app.node_used += 1;
        const right_child_idx = app.node_used;
        app.node_used += 1;

        var left_child_node = &app.bvh_nodes[left_child_idx];
        left_child_node.left_first = node.left_first;
        left_child_node.tri_count = left_count;

        var right_child_node = &app.bvh_nodes[right_child_idx];
        right_child_node.left_first = i;
        right_child_node.tri_count = node.tri_count - left_count;

        node.left_first = left_child_idx;
        node.tri_count = 0;

        app.updateNodeBounds(left_child_idx);
        app.updateNodeBounds(right_child_idx);

        // Recourse
        app.subdivide(left_child_idx);
        app.subdivide(right_child_idx);
    }

    fn findBestSplitPlane(app: *BasicBVHApp, node: *BVHNode, axis: *u32, split_pos: *f32) f32 {
        var best_cost: f32 = 1.0e+30;
        var a: u32 = 0;
        while (a < 3) : (a += 1) {
            var bounds_min: f32 = 1.0e+30;
            var bounds_max: f32 = -1.0e+30;
            var i: u32 = 0;
            while (i < node.tri_count) : (i += 1) {
                const triangle = &app.triangles[app.triangle_indices[node.left_first + i]];
                bounds_min = std.math.min(bounds_min, triangle.centroid[a]);
                bounds_max = std.math.max(bounds_max, triangle.centroid[a]);
            }

            if (bounds_min == bounds_max) {
                continue;
            }

            var bin: [num_bins]Bin = undefined;
            i = 0;
            while (i < num_bins) : (i += 1) {
                bin[i].tri_count = 0;
                bin[i].bounds = AABB.init();
            }

            var scale = @intToFloat(f32, num_bins) / (bounds_max - bounds_min);
            i = 0;
            while (i < node.tri_count) : (i += 1) {
                const triangle = &app.triangles[app.triangle_indices[node.left_first + i]];
                const bin_index: u32 = std.math.min(num_bins - 1, @floatToInt(u32, (triangle.centroid[a] - bounds_min) * scale));
                bin[bin_index].tri_count += 1;
                bin[bin_index].bounds.grow(triangle.vertex0);
                bin[bin_index].bounds.grow(triangle.vertex1);
                bin[bin_index].bounds.grow(triangle.vertex2);
            }

            // Gather data for the num_bins - 1 planes between the num_bins bins
            var left_area: [num_bins - 1]f32 = undefined;
            var right_area: [num_bins - 1]f32 = undefined;
            var left_count: [num_bins - 1]u32 = undefined;
            var right_count: [num_bins - 1]u32 = undefined;
            var left_box = AABB.init();
            var right_box = AABB.init();
            var left_sum: u32 = 0;
            var right_sum: u32 = 0;
            i = 0;
            while (i < num_bins - 1) : (i += 1) {
                left_sum += bin[i].tri_count;
                left_count[i] = left_sum;
                left_box.growWithAABB(&bin[i].bounds);
                left_area[i] = left_box.area();

                right_sum += bin[num_bins - 1 - i].tri_count;
                right_count[num_bins - 2 - i] = right_sum;
                right_box.growWithAABB(&bin[num_bins - 1 - i].bounds);
                right_area[num_bins - 2 - i] = right_box.area();
            }

            // Calculate SAH costs for num_bins - 1 planes
            scale = (bounds_max - bounds_min) / @intToFloat(f32, num_bins);
            i = 0;
            while (i < num_bins - 1) : (i += 1) {
                const plane_cost = @intToFloat(f32, left_count[i]) * left_area[i] + @intToFloat(f32, right_count[i]) * right_area[i];
                if (plane_cost < best_cost) {
                    axis.* = a;
                    split_pos.* = bounds_min + scale * @intToFloat(f32, i + 1);
                    best_cost = plane_cost;
                }
            }
        }

        return best_cost;
    }

    pub fn evaluateSAH(app: *BasicBVHApp, node: *BVHNode, axis: u32, pos: f32) f32 {
        // Determine triangle counts and bounds for this split candidate
        var left_box = AABB.init();
        var right_box = AABB.init();
        var left_count: u32 = 0;
        var right_count: u32 = 0;

        var i: u32 = 0;
        while (i < node.tri_count) : (i += 1) {
            const triangle = &app.triangles[app.triangle_indices[node.left_first + i]];
            if (triangle.centroid[axis] < pos) {
                left_count += 1;
                left_box.grow(triangle.vertex0);
                left_box.grow(triangle.vertex1);
                left_box.grow(triangle.vertex2);
            } else {
                right_count += 1;
                right_box.grow(triangle.vertex0);
                right_box.grow(triangle.vertex1);
                right_box.grow(triangle.vertex2);
            }
        }

        const cost: f32 = @intToFloat(f32, left_count) * left_box.area() + @intToFloat(f32, right_count) * right_box.area();
        if (cost > 0)
        {
            return cost;
        }

        return 1.0e+30;
    }
};

pub fn main() anyerror!void {
    _ = c.SDL_Init(c.SDL_INIT_VIDEO);
    defer c.SDL_Quit();

    const width: u32 = 640;
    const height: u32 = 640;

    var window = c.SDL_CreateWindow("BVH", c.SDL_WINDOWPOS_CENTERED, c.SDL_WINDOWPOS_CENTERED, width, height, 0);
    defer c.SDL_DestroyWindow(window);

    var renderer = c.SDL_CreateRenderer(window, 0, c.SDL_RENDERER_PRESENTVSYNC);
    defer c.SDL_DestroyRenderer(renderer);

    var rmask: u32 = 0;
    var gmask: u32 = 0;
    var bmask: u32 = 0;
    var amask: u32 = 0;

    // SDL interprets each pixel as a 32-bit number, so our masks must depend
    // on the endianness (byte order) of the machine
    switch(endianness) {
        .Big => {
            rmask = 0xff000000;
            gmask = 0x00ff0000;
            bmask = 0x0000ff00;
            amask = 0x000000ff;
        },
        .Little => {
            rmask = 0x000000ff;
            gmask = 0x0000ff00;
            bmask = 0x00ff0000;
            amask = 0xff000000;
        }
    }

    var surface = c.SDL_CreateRGBSurface(0, width, height, 32, rmask, gmask, bmask, amask);
    defer c.SDL_FreeSurface(surface);

    var texture_rect: c.SDL_Rect = undefined;
    texture_rect.x = 0;
    texture_rect.y = 0;
    texture_rect.w = width;
    texture_rect.h = height;

    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var bvh_app = BasicBVHApp.init(allocator);
    defer bvh_app.deinit(allocator);

    const pixel_loop_marker = "pixel loop";
    const pixel_color_marker = "pixel set color";
    const intersect_marker = "intersect BVH";

    mainloop: while (true) {
        var sdl_event: c.SDL_Event = undefined;
        while (c.SDL_PollEvent(&sdl_event) != 0) {
            switch (sdl_event.type) {
                c.SDL_QUIT => break :mainloop,
                else => {}
            }
        }

        _ = c.SDL_SetRenderDrawColor(renderer, 0xff, 0x00, 0x00, 0x00);
        _ = c.SDL_RenderClear(renderer);

        // Clear surface
        _ = c.SDL_FillRect(surface, null, c.SDL_MapRGB(surface.*.format, 0, 0, 0));

        const p0: [3]f32 = .{ -1, 1, 2 };
        const p1: [3]f32 = .{ 1, 1, 2 };
        const p2: [3]f32 = .{ -1, -1, 2 };
        var ray: Ray = undefined;
        ray.origin = .{ 0, 3.5, -4.5 };

        const start = std.time.milliTimestamp();

        bvh_app.animate();
        bvh_app.refitBVH();

        {
            const pixels_loop_tracy_zone = ztracy.ZoneNC(@src(), pixel_loop_marker, 0x00_ff_00_00);
            defer pixels_loop_tracy_zone.End();

            var tile: u32 = 0;
            while (tile < 6400) : (tile += 1) {
                var x: u32 = tile % 80;
                var y: u32 = tile / 80;
                var v: u32 = 0;
                while (v < 8) : (v += 1) {
                    var u: u32 = 0;
                    while (u < 8) : (u += 1) {
                        const pixel_pos = calculatePixelPosition(ray.origin, p0, p1, p2, x * 8 + u, y * 8 + v, width, height);
                        ray.direction = float3Normalize(float3Sub(pixel_pos, ray.origin));
                        ray.r_direction = .{ 1.0 / ray.direction[0], 1.0 / ray.direction[1], 1.0 / ray.direction[2] };
                        ray.t = 1.0e+30;

                        const intersect_tracy_zone = ztracy.ZoneNC(@src(), intersect_marker, 0x00_00_ff_00);
                        bvh_app.intersectBVH(&ray, bvh_app.root_node_index);
                        intersect_tracy_zone.End();

                        if (ray.t < 1.0e+30) {
                            const pixels_color_tracy_zone = ztracy.ZoneNC(@src(), pixel_color_marker, 0x00_ff_00_00);
                            defer pixels_color_tracy_zone.End();

                            var d = 255 - @floatToInt(u32, (ray.t - 4) * 180);
                            // var d: u32 = 255;
                            var color: u32 = 0xff000000;
                            color |= d << 0;
                            color |= d << 8;
                            color |= d << 16;
                            setPixelColor(surface, x * 8 + u, y * 8 + v, color);
                        }
                    }
                }
            }
        }

        const end = std.time.milliTimestamp();
        const elapsed: f32 = @intToFloat(f32, end - start);

        print("tracing time: {d:.2}ms ({d:2.2}M rays/s)", .{elapsed, @intToFloat(f32, width * height) / elapsed / 1000});

        var texture = c.SDL_CreateTextureFromSurface(renderer, surface);
        _ = c.SDL_RenderCopy(renderer, texture, null, &texture_rect);

        c.SDL_RenderPresent(renderer);
        ztracy.FrameMark();
    }
}

fn calculatePixelPosition(origin: [3]f32, p0: [3]f32, p1: [3]f32, p2: [3]f32, x: u32, y: u32, width: u32, height: u32) [3]f32 {
    var a = float3Sub(p1, p0);
    var b = float3Sub(p2, p0);
    var w = @intToFloat(f32, x) / @intToFloat(f32, width);
    var h = @intToFloat(f32, y) / @intToFloat(f32, height);

    return float3Add(float3Add(origin, p0), float3Add(float3MulScalar(a, w), float3MulScalar(b, h)));
}

fn setPixelColor(surface: *c.SDL_Surface, x: u32, y: u32, pixel: u32) void {
    const data_addr = @alignCast(4, @ptrCast([*]u8, surface.*.pixels) + y * @intCast(u32, surface.*.pitch) + x * @intCast(u32, surface.*.format.*.BytesPerPixel));
    const target_pixel = @ptrCast(*u32, data_addr);
    target_pixel.* = pixel;
}