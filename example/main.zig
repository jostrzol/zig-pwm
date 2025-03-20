const std = @import("std");
const pwm = @import("pwm");
const signal = @cImport(@cInclude("signal.h"));

const pwm_channel: u8 = 1; // gpio 13
const period_ms: u64 = 5000;
const pwm_updates_per_period: u64 = 50;
const pwm_min: f32 = 0.2;
const pwm_max: f32 = 1.0;
const pwm_frequency: f32 = 1000;
const sleep_time_ns: u64 = period_ms * std.time.ns_per_ms / pwm_updates_per_period;

var do_continue = true;
pub fn interrupt_handler(_: c_int) callconv(.C) void {
    std.debug.print("\nGracefully stopping\n", .{});
    do_continue = false;
}
const interrupt_sigaction = signal.struct_sigaction{
    .__sigaction_handler = .{ .sa_handler = &interrupt_handler },
};

pub fn main() !void {
    if (signal.sigaction(signal.SIGINT, &interrupt_sigaction, null) != 0) {
        return error.SigactionNotSet;
    }

    var chip = try pwm.Chip.init(0);
    defer chip.deinit();

    std.debug.print("Controlling motor from Zig.\n", .{});

    var channel = try chip.channel(pwm_channel);
    defer channel.deinit();

    // Needed to set other parameters.
    try channel.setParameters(.{
        .frequency = pwm_frequency,
        .duty_cycle_ratio = 0,
    });

    try channel.disable();
    std.log.debug("enabled?: {}", .{try channel.isEnabled()});

    try channel.setPolarity(pwm.Polarity.normal);
    std.log.debug("polarity: {}", .{try channel.getPolarity()});
    try channel.setPolarity(pwm.Polarity.inversed);
    std.log.debug("polarity: {}", .{try channel.getPolarity()});
    try channel.setPolarity(pwm.Polarity.normal);

    try channel.enable();
    std.log.debug("enabled?: {}", .{try channel.isEnabled()});

    main_loop: while (true) {
        for (0..pwm_updates_per_period) |i| {
            updateDutyCycle(channel, i) catch |err| {
                std.debug.print("error updating duty cycle: {}", .{err});
            };

            if (!do_continue) break :main_loop;
            std.time.sleep(sleep_time_ns);
        }
    }
}

fn updateDutyCycle(channel: *pwm.Channel, step: u64) !void {
    const sin_arg_ratio = @as(f32, @floatFromInt(step)) / pwm_updates_per_period;
    const sin = std.math.sin(sin_arg_ratio * 2 * std.math.pi);
    const ratio = (sin + 1) / 2;
    const duty_cycle = pwm_min + ratio * (pwm_max - pwm_min);
    try channel.setParameters(.{
        .frequency = null,
        .duty_cycle_ratio = duty_cycle,
    });

    std.log.debug(
        "period [ns]: {}, duty_cycle [ns]: {}",
        .{ try channel.getPeriodNs(), try channel.getDutyCycleNs() },
    );
}
