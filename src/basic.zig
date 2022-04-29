const std = @import("std");
const endianness = @import("builtin").target.cpu.arch.endian();
const c = @cImport({
    @cInclude("SDL.h");
});

const math = std.math;

const Tri = struct {
    vertex0: [3]f32,
    vertex1: [3]f32,
    vertex2: [3]f32,
    centroid: [3]f32,
};

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

const BVHNode = struct {
    aabb_min: [3]f32,
    aabb_max: [3]f32,
    left_first: u32,
    tri_count: u32,

    pub fn is_leaf(node: *BVHNode) bool {
        return node.tri_count > 0;
    }
};

const Ray = struct {
    origin: [3]f32,
    direction: [3]f32,
    t: f32 = 1.0e+30,
};

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

pub fn intersectAABB(ray: *Ray, bmin: [3]f32, bmax: [3]f32) bool {
    var tx1: f32 = (bmin[0] - ray.origin[0]) / ray.direction[0];
    var tx2: f32 = (bmax[0] - ray.origin[0]) / ray.direction[0];
    var tmin: f32 = math.min(tx1, tx2);
    var tmax: f32 = math.max(tx1, tx2);

    var ty1: f32 = (bmin[1] - ray.origin[1]) / ray.direction[1];
    var ty2: f32 = (bmax[1] - ray.origin[1]) / ray.direction[1];
    tmin = math.max(tmin, math.min(ty1, ty2));
    tmax = math.min(tmax, math.max(ty1, ty2));

    var tz1: f32 = (bmin[2] - ray.origin[2]) / ray.direction[2];
    var tz2: f32 = (bmax[2] - ray.origin[2]) / ray.direction[2];
    tmin = math.max(tmin, math.min(tz1, tz2));
    tmax = math.min(tmax, math.max(tz1, tz2));

    return tmax >= tmin and tmin < ray.t and tmax > 0;
}


const BasicBVHApp = struct {
    num_triangles: u32 = 0,
    triangles: std.ArrayListUnmanaged(Tri),
    triangle_indices: std.ArrayListUnmanaged(u32),
    bvh_nodes: std.ArrayListUnmanaged(BVHNode),
    root_node_index: u32 = 0,

    pub fn init(allocator: std.mem.Allocator) BasicBVHApp {
        var app: BasicBVHApp = undefined;
        app.root_node_index = 0;

        // app.generateRandomTriangleSoup(allocator);
        app.loadTrianglesFromMesh(allocator) catch unreachable;
        app.buildBVH(allocator);

        return app;
    }

    pub fn deinit(app: *BasicBVHApp, allocator: std.mem.Allocator) void {
        app.triangles.deinit(allocator);
        app.triangle_indices.deinit(allocator);
        app.bvh_nodes.deinit(allocator);
    }

    fn generateRandomTriangleSoup(app: *BasicBVHApp, allocator: std.mem.Allocator) void {
        app.num_triangles = 64;

        app.triangles = std.ArrayListUnmanaged(Tri).initCapacity(allocator, app.num_triangles) catch unreachable;
        app.triangle_indices = std.ArrayListUnmanaged(u32).initCapacity(allocator, app.num_triangles) catch unreachable;

        var prng = std.rand.DefaultPrng.init(0x12345678);
        const random = prng.random();

        var i: u32 = 0;
        while (i < app.num_triangles) : (i += 1) {
            var r0: [3]f32 = .{ random.float(f32), random.float(f32), random.float(f32) };
            var r1: [3]f32 = .{ random.float(f32), random.float(f32), random.float(f32) };
            var r2: [3]f32 = .{ random.float(f32), random.float(f32), random.float(f32) };

            var triangle: Tri = undefined;
        
            triangle.vertex0 = float3Sub(float3MulScalar(r0, 9), .{ 5.0, 5.0, 5.0 });
            triangle.vertex1 = float3Add(triangle.vertex0, r1);
            triangle.vertex2 = float3Add(triangle.vertex0, r2);

            app.triangles.appendAssumeCapacity(triangle);
        }
    }

    fn loadTrianglesFromMesh(app: *BasicBVHApp, allocator: std.mem.Allocator) !void {
        const dir = std.fs.cwd();
        var file = try dir.openFile("content/unity.tri", .{ .mode = .read_only });

        var reader = file.reader();
        var buffer: [512]u8 = undefined;
        app.num_triangles = 0;
        while (try reader.readUntilDelimiterOrEof(buffer[0..], '\n')) |_| {
            app.num_triangles += 1;
        }
        std.log.debug("Num triangles found in file: {}", .{app.num_triangles});
        file.close();

        app.triangles = std.ArrayListUnmanaged(Tri).initCapacity(allocator, app.num_triangles) catch unreachable;
        app.triangle_indices = std.ArrayListUnmanaged(u32).initCapacity(allocator, app.num_triangles) catch unreachable;

        // TODO: Figure out a way to rewind the file reader instead of re-opening the file again
        file = try dir.openFile("content/unity.tri", .{ .mode = .read_only });
        defer file.close();
        reader = file.reader();
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

            app.triangles.appendAssumeCapacity(triangle);
        }
    }

    pub fn intersectBVH(bvh: *BasicBVHApp, ray: *Ray, node_index: u32) void {
        var node = &bvh.bvh_nodes.items[node_index];
        if (!intersectAABB(ray, node.aabb_min, node.aabb_max)) {
            return;
        }

        if (node.is_leaf()) {
            var i: u32 = 0;
            while (i < node.tri_count) : (i += 1) {
                intersectTri(ray, &bvh.triangles.items[bvh.triangle_indices.items[node.left_first + i]]);
            }
        } else {
            bvh.intersectBVH(ray, node.left_first);
            bvh.intersectBVH(ray, node.left_first + 1);
        }
    }

    fn buildBVH(app: *BasicBVHApp, allocator: std.mem.Allocator) void {
        std.debug.assert(app.num_triangles > 0);
        app.bvh_nodes = std.ArrayListUnmanaged(BVHNode).initCapacity(allocator, app.num_triangles * 2) catch unreachable;

        std.debug.assert(app.num_triangles > 0);
        std.debug.assert(app.root_node_index == 0);

        // Populate triangle index array
        var i: u32 = 0;
        while (i < app.num_triangles) : (i += 1) {
            app.triangle_indices.appendAssumeCapacity(i);
        }

        // Calculate triangle centroids for partitioning
        i = 0;
        while (i < app.num_triangles) : (i += 1) {
            app.triangles.items[i].centroid = .{
                (app.triangles.items[i].vertex0[0] + app.triangles.items[i].vertex1[0] + app.triangles.items[i].vertex2[0]) * 0.3333,
                (app.triangles.items[i].vertex0[1] + app.triangles.items[i].vertex1[1] + app.triangles.items[i].vertex2[1]) * 0.3333,
                (app.triangles.items[i].vertex0[2] + app.triangles.items[i].vertex1[2] + app.triangles.items[i].vertex2[2]) * 0.3333,
            };
        }

        // Assign all triangles to the root node
        var bvh_node: BVHNode = undefined;
        bvh_node.left_first = 0;
        bvh_node.tri_count = app.num_triangles;

        app.bvh_nodes.appendAssumeCapacity(bvh_node);

        app.updateNodeBounds(app.root_node_index);

        // Subdivide recursively
        app.subdivide(app.root_node_index);
    }

    fn updateNodeBounds(app: *BasicBVHApp, node_index: u32) void {
        std.debug.assert(node_index < app.bvh_nodes.items.len);

        var node = &app.bvh_nodes.items[node_index];
        node.aabb_min = .{ 1.0e+30, 1.0e+30, 1.0e+30 };
        node.aabb_max = .{ -1.0e+30, -1.0e+30, -1.0e+30 };

        var i: u32 = 0;
        const first: u32 = node.left_first;

        while (i < node.tri_count) : (i += 1) {
            var left_tri_idx = app.triangle_indices.items[first + i];
            const left_tri = app.triangles.items[left_tri_idx];

            node.aabb_min = float3Min(node.aabb_min, left_tri.vertex0);
            node.aabb_min = float3Min(node.aabb_min, left_tri.vertex1);
            node.aabb_min = float3Min(node.aabb_min, left_tri.vertex2);
            node.aabb_max = float3Max(node.aabb_max, left_tri.vertex0);
            node.aabb_max = float3Max(node.aabb_max, left_tri.vertex1);
            node.aabb_max = float3Max(node.aabb_max, left_tri.vertex2);
        }
    }

    fn subdivide(app: *BasicBVHApp, node_index: u32) void {
        std.debug.assert(node_index < app.bvh_nodes.items.len);

        // Terminate recursion
        var node = &app.bvh_nodes.items[node_index];
        if (node.tri_count <= 2) {
            return;
        }

        // Determine split axis and position
        const extent: [3]f32 = .{
            node.aabb_max[0] - node.aabb_min[0],
            node.aabb_max[1] - node.aabb_min[1],
            node.aabb_max[2] - node.aabb_min[2],
        };

        var axis: u32 = 0;
        if (extent[1] > extent[0]) {
            axis = 1;
        }

        if (extent[2] > extent[axis]) {
            axis = 2;
        }

        const split_pos: f32 = node.aabb_min[axis] + extent[axis] * 0.5;

        // In-place partition
        var i: u32 = node.left_first;
        var j: u32 = i + node.tri_count - 1;
        while (i <= j) {
            if (app.triangles.items[app.triangle_indices.items[i]].centroid[axis] < split_pos) {
                i += 1;
            } else {
                var tmp = app.triangle_indices.items[i];
                app.triangle_indices.items[i] = app.triangle_indices.items[j];
                app.triangle_indices.items[j] = tmp;
                j -= 1;
            }
        }

        // Abort split if one of the sides is empty
        const left_count: u32 = i - node.left_first;
        if (left_count == 0 or left_count == node.tri_count) {
            return;
        }

        // Create child nodes
        const left_child_idx: u32 = @intCast(u32, app.bvh_nodes.items.len);
        var left_child_node: BVHNode = undefined;
        left_child_node.left_first = node.left_first;
        left_child_node.tri_count = left_count;
        app.bvh_nodes.appendAssumeCapacity(left_child_node);

        const right_child_idx: u32 = @intCast(u32, app.bvh_nodes.items.len);
        var right_child_node: BVHNode = undefined;
        right_child_node.left_first = i;
        right_child_node.tri_count = node.tri_count - left_count;
        app.bvh_nodes.appendAssumeCapacity(right_child_node);

        node.left_first = left_child_idx;
        node.tri_count = 0;

        app.updateNodeBounds(left_child_idx);
        app.updateNodeBounds(right_child_idx);

        // Recourse
        app.subdivide(left_child_idx);
        app.subdivide(right_child_idx);
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
        ray.origin = .{ -1.5, -0.2, -2.5 };

        const start = std.time.milliTimestamp();

        var y: u32 = 0;
        const inv_far_plane: f32 = 1.0 / 5.0;

        while (y < height) : (y += 1) {
            var x: u32 = 0;
            while (x < width) : (x += 1) {
                const pixel_pos = calculatePixelPosition(ray.origin, p0, p1, p2, x, y, width, height);
                ray.direction = float3Normalize(float3Sub(pixel_pos, ray.origin));
                ray.t = 1.0e+30;

                bvh_app.intersectBVH(&ray, bvh_app.root_node_index);

                if (ray.t < 1.0e+30) {
                    var d = @floatToInt(u32, (1.0 - ray.t * inv_far_plane) * 255.99);
                    var color: u32 = 0xff000000;
                    color |= d << 0;
                    color |= d << 8;
                    color |= d << 16;
                    setPixelColor(surface, x, y, color);
                }
            }
        }

        const end = std.time.milliTimestamp();
        const elapsed: f32 = @intToFloat(f32, end - start);

        std.log.debug("tracing time: {d:.2}ms ({d:2.2}M rays/s)", .{elapsed, @intToFloat(f32, width * height) / elapsed / 1000});

        var texture = c.SDL_CreateTextureFromSurface(renderer, surface);
        _ = c.SDL_RenderCopy(renderer, texture, null, &texture_rect);

        c.SDL_RenderPresent(renderer);
    }
}

fn calculatePixelPosition(origin: [3]f32, p0: [3]f32, p1: [3]f32, p2: [3]f32, x: u32, y: u32, width: u32, height: u32) [3]f32 {
    return float3Add(origin, float3Add(p0, float3Add(
        float3MulScalar(float3Sub(p1, p0), (@intToFloat(f32, x) / @intToFloat(f32, width))),
        float3MulScalar(float3Sub(p2, p0), @intToFloat(f32, y) / @intToFloat(f32, height)))));
}

fn setPixelColor(surface: *c.SDL_Surface, x: u32, y: u32, pixel: u32) void {
    const data_addr = @alignCast(4, @ptrCast([*]u8, surface.*.pixels) + y * @intCast(u32, surface.*.pitch) + x * @intCast(u32, surface.*.format.*.BytesPerPixel));
    const target_pixel = @ptrCast(*u32, data_addr);
    target_pixel.* = pixel;
}