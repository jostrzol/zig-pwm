# Zig PWM

Control hardware PWM chips through Linux Kernel PWM sysfs interface.

## Installing

First add the `zig-pwm` dependency to your `build.zig.zon` via:

```sh
zig fetch --save https://github.com/jostrzol/zig-pwm/archive/refs/tags/v0.0.1.tar.gz
```

You can change the library version in the link.

Then add the `zig-pwm` module to your output target like below:

```zig
{
pub fn build(b: *std.Build) void {
    const optimize = b.standardOptimizeOption(.{});
    const target = b.standardTargetOptions(.{});

    const zig_pwm = b.dependency("zig-pwm", .{ .target = target, .optimize = optimize });
    
    const exe = b.addExecutable(.{
        .name = "2-motor-controller-zig",
        .root_source_file = b.path("./main.zig"),
        .target = target,
        .optimize = optimize,
    });
    exe.root_module.addImport("pwm", zig_pwm.module("pwm"));

    b.installArtifact(exe);
}
```

## Usage

See [example program](./example/main.zig).

## Compiling for Raspberry PI Zero

If standard compilation options are registered in your `build.zig`:

```zig
pub fn build(b: *std.Build) !void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    // ...
}
```

, then just run:

```sh
zig build install -Dtarget=arm-linux-gnueabihf -Dcpu=arm1176jzf_s
```

> [!NOTE]
> For other Raspberry PI chips you might have to change the cpu argument for the
> correct one.

## Compiling example for Raspberry PI Zero

```sh
zig build example -Dtarget=arm-linux-gnueabihf -Dcpu=arm1176jzf_s
```

> [!NOTE]
> For other Raspberry PI chips you might have to change the cpu argument for the
> correct one.

## References

- [Linux Kernel PWM sysfs interface documentation](https://www.kernel.org/doc/html/v5.10/driver-api/pwm.html)
