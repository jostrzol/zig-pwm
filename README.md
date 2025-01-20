# Zig PWM

Control hardware PWM chips through Linux Kernel PWM sysfs interface.

## Installing

Add

```zig
{
    // ...

    .dependencies = .{
        .pwm = .{
            .url = "https://github.com/jostrzol/zig-pwm/archive/refs/tags/v0.0.1.tar.gz",
            .hash = "TODO", // TODO
        },
    },
}
```

to your `build.zig.zon`.

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
zig build <your_install_step_name> -Dtarget=arm-linux-gnueabihf -Dcpu=arm1176jzf_s
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
