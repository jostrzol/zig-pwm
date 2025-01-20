const std = @import("std");

pub fn build(b: *std.Build) !void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    // Add the pwm module so it can be used by the package manager
    const module = b.createModule(.{ .root_source_file = b.path("src/index.zig") });
    try b.modules.put(b.dupe("pwm"), module);

    // Example
    const example_step = b.step("example", "Build the example program");

    const example_exe = b.addExecutable(.{
        .name = "example",
        .root_source_file = b.path("./example/main.zig"),
        .target = target,
        .optimize = optimize,
    });
    example_exe.root_module.addImport("pwm", module);
    example_exe.linkLibC();

    const build_step = b.addInstallArtifact(example_exe, .{});
    example_step.dependOn(&build_step.step);
}
