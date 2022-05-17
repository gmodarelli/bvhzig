const std = @import("std");
const ztracy = @import("3rdparty/ztracy/build.zig");

const content_dir = "content/";

pub fn build(b: *std.build.Builder) void {
    // Standard target options allows the person running `zig build` to choose
    // what target to build for. Here we do not override the defaults, which
    // means any target is allowed, and the default is native. Other options
    // for restricting supported target set are available.
    const target = b.standardTargetOptions(.{});

    // Standard release options allow the person running `zig build` to select
    // between Debug, ReleaseSafe, ReleaseFast, and ReleaseSmall.
    const mode = b.standardReleaseOptions();

    const enable_tracy = b.option(bool, "enable-tracy", "Enable Tracy profiler") orelse false;

    const exe_options = b.addOptions();
    exe_options.addOption(bool, "enable_tracy", enable_tracy);

    const exe = b.addExecutable("main", "src/main.zig");
    exe.setTarget(target);
    exe.setBuildMode(mode);
    exe.addOptions("build_options", exe_options);

    const options_pkg = exe_options.getPackage("build_options");
    exe.addPackage(ztracy.getPkg(b, options_pkg));
    ztracy.link(exe, enable_tracy);

    const install_content_step = b.addInstallDirectory(.{
        .source_dir = thisDir() ++ "/" ++ content_dir,
        .install_dir = .{ .custom = "" },
        .install_subdir = "bin/" ++ content_dir,
    });

    exe.step.dependOn(&install_content_step.step);

    // SDL 2
    const sdl_path = ".\\3rdparty\\SDL2-2.0.20\\";
    exe.addIncludeDir(sdl_path ++ "include");
    exe.addLibPath(sdl_path ++ "lib\\x64");
    b.installBinFile(sdl_path ++ "lib\\x64\\SDL2.dll", "SDL2.dll");
    exe.linkSystemLibrary("sdl2");
    exe.linkLibC();

    exe.install();

    const run_cmd = exe.run();
    run_cmd.step.dependOn(b.getInstallStep());
    if (b.args) |args| {
        run_cmd.addArgs(args);
    }

    const run_step = b.step("run", "Run the app");
    run_step.dependOn(&run_cmd.step);
}

fn thisDir() []const u8 {
    return std.fs.path.dirname(@src().file) orelse ".";
}