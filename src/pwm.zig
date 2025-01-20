const std = @import("std");

/// Path to root of Linux Kernel PWM interface.
const PWM_ROOT_PATH = "/sys/class/pwm";
/// Maximum number of channels handled by the library.
const MAX_CHANNELS = 8;
/// Maximum length of path in Linux Kernel PWM interface file structure handled
/// by the library.
const MAX_PWM_PATH_BYTES = 128;
/// Characters considered whitespace when reading from the Linux Kernel PWM
/// interface.
const WHITESPACE = [_]u8{
    ' ', // Space
    '\t', // Horizontal Tab
    '\n', // Line Feed
    '\r', // Carriage Return
    '\x00', // Null ('\0')
    '\x0B', // Vertical Tab ('\v')
    '\x0C', // Form Feed ('\f')
};

/// Buffer for path in sysfs PWM driver structure.
const PathPwmBuffer = [MAX_PWM_PATH_BYTES]u8;

/// Represents a Linux Kernel PWM chip interface, for interacting with a driver
/// for the hardware PWM components.
///
/// The chip's root file is located under `/sys/class/pwm/pwmchip{Chip.number}`.
///
/// ---
/// **Note:** It may be required to enable the Linux Kernel PWM interface for
/// the system. For example, Raspberry PI requires appending either:
///
/// * `dtoverlay=pwm` for only one channel of PWM,
/// * or `dtoverlay=pwm-2chan` for both channels,
///
/// to either:
///
/// * `/boot/firmware/config.txt` on Raspberry Pi OS,
/// * or `boot/firmware/usercfg.txt` on Ubuntu.
pub const Chip = struct {
    /// Represents chip's Linux Kernel interface files opened by the library.
    ///  ---
    /// **Note:** does not contain `/sys/class/pwm/pwmchip{number}/npwn` file,
    /// because it is accessed only once and its result is cached.
    const FilesOpened = struct {
        /// Handle to file `/sys/class/pwm/pwmchip{number}/export`.
        export_: ?std.fs.File = null,
        /// Handle to file `/sys/class/pwm/pwmchip{number}/unexport`.
        unexport: ?std.fs.File = null,
    };

    /// Determines which one of the chips located at `/sys/class/pwm/*` to
    /// access.
    number: u8,
    /// **This field should not be accessed manualy. See `Chip.channel`.**
    ///
    /// Contains `Channel` objects, used to control the actual hardware PWM
    /// channels.
    ///
    /// Channel objects are instantinated lazily, so this array contains `null`s
    /// until the channel is first accessed. Upon channel's deinitialization it
    /// becomes null again.
    channels: [MAX_CHANNELS]?Channel,

    /// **This field should not be accessed manualy. See `Chip.npwm`.**
    ///
    /// Containes cached value of the npwm value for the given chip, under
    /// `/sys/class/pwm/pwmchip{number}/npwm`.
    npwm_cached: ?u8 = null,
    /// **This field should not be accessed manualy. See `Chip.file`.**
    ///
    /// Containes file handles to already opened files under this chip. The
    /// handles are reused to speed up the access.
    files_opened: FilesOpened = .{},

    /// Initializes the chip. Does not interact with the PWN interface yet.
    ///
    /// * `chip_nr` - determines which one of the chips located at
    ///   `/sys/class/pwm/*` to access.
    pub fn init(chip_nr: u8) !Chip {
        return .{
            .number = chip_nr,
            .channels = .{null} ** MAX_CHANNELS,
        };
    }

    /// Deinitializes the chip and all of its opened channels.
    pub fn deinit(self: *Chip) void {
        for (self.channels) |maybe_chan| {
            var chan = maybe_chan orelse continue;
            chan.deinit();
        }
        inline for (std.meta.fields(FilesOpened)) |f| {
            const maybe_file = @field(self.files_opened, f.name);
            if (maybe_file) |file_| file_.close();
        }
    }

    /// Returns number of PWM channels that this chip controls.
    ///
    /// The value is read from `/sys/class/pwm/pwmchip{self.number}/npwm`. It is
    /// read only once, and then remembered in `self.npwm_cached`.
    pub fn npwm(self: *Chip) !u8 {
        if (self.npwm_cached) |value| return value;

        var buffer: PathPwmBuffer = undefined;
        const path_ = try self.path(&buffer, "/npwm", .{});
        const file_ = try std.fs.openFileAbsolute(path_, .{ .mode = .read_only });
        defer file_.close();

        const value = try readInt(u8, 8, file_, 10);
        self.npwm_cached = value;
        return value;
    }

    /// Returns a `Channel` object for controlling one of the actual hardware
    /// PWM channels. The channel returned is guaranteed to be exported.
    ///
    /// The `Channel` object is stored by `self`. Multiple calls to this function
    /// with same `channel_nr` will result in the same object beeing return.
    ///
    /// * `channel_nr` - determines which one of the channels located at
    ///   `/sys/class/pwm/pwmchip{self.number}/pwm*` to access.
    pub fn channel(self: *Chip, channel_nr: u8) !*Channel {
        if (channel_nr > try self.npwm()) return error.ChannelOutOfRange;

        if (self.channels[channel_nr]) |_| return &self.channels[channel_nr].?;

        self.channels[channel_nr] = try Channel.init(self, channel_nr);
        return &self.channels[channel_nr].?;
    }

    /// Export the chip's PWM channel number `channel_nr`.
    fn export_(self: *Chip, channel_nr: u8) !void {
        const file_ = try self.file(.export_, .{ .mode = .write_only });
        try file_.writer().print("{}", .{channel_nr});
    }

    /// Unexport the chip's PWM channel number `channel_nr`.
    fn unexport(self: *Chip, channel_nr: u8) !void {
        const file_ = try self.file(.unexport, .{ .mode = .write_only });
        try file_.writer().print("{}", .{channel_nr});
    }

    /// Opens file corresponding to the `location` and returns its handle. If
    /// the file has been already accessed before, the old handle is returned
    /// without opening the file for the second time.
    ///
    /// Is not used to access `/sys/class/pwm/pwmchip{self.number}/npwm`, because
    /// this file's handle is not cached.
    ///
    /// * `location` - which of the chip's file to open (see
    ///   `Chip.FilesOpened`).
    fn file(
        self: *Chip,
        comptime location: std.meta.FieldEnum(FilesOpened),
        flags: std.fs.File.OpenFlags,
    ) !std.fs.File {
        const name = @tagName(location);
        if (@field(self.files_opened, name)) |file_| return file_;

        var buffer: PathPwmBuffer = undefined;
        const path_ = try self.path(&buffer, "/" ++ stripUnderscore(name), .{});
        const file_ = try std.fs.openFileAbsolute(path_, flags);
        @field(self.files_opened, name) = file_;
        return file_;
    }

    /// Prints the path in format: `/sys/class/pwm/pwmchip{self.number}{fmt}`
    /// into the `buffer`.
    fn path(
        self: *const Chip,
        buffer: []u8,
        comptime fmt: []const u8,
        args: anytype,
    ) std.fmt.BufPrintError![]u8 {
        const buf = try std.fmt.bufPrint(
            buffer,
            PWM_ROOT_PATH ++ "/pwmchip{}" ++ fmt,
            .{self.number} ++ args,
        );
        return buf;
    }
};

/// Represents a chip's PWM channel, which is responsible for actually
/// generating the PWM wave.
///
/// To use the channel, set its period and duty cycle (preferably through
/// `Channel.setParameters`) and enable it through `Channel.enable`.
///
/// ---
/// **Note:** The channel's frequency/period should be the first thing to set.
/// Other setters will refuse to work otherwise
pub const Channel = struct {
    /// Represents channel's Linux Kernel interface files opened by the library.
    const FilesOpened = struct {
        /// Handle to file
        /// `/sys/class/pwm/pwmchip{chip.number}/pwm{number}/period`.
        period: ?std.fs.File = null,
        /// Handle to file
        /// `/sys/class/pwm/pwmchip{chip.number}/pwm{number}/duty_cycle`.
        duty_cycle: ?std.fs.File = null,
        /// Handle to file
        /// `/sys/class/pwm/pwmchip{chip.number}/pwm{number}/polarity`.
        polarity: ?std.fs.File = null,
        /// Handle to file
        /// `/sys/class/pwm/pwmchip{chip.number}/pwm{number}/enable`.
        enable: ?std.fs.File = null,
    };

    /// The parent chip of this channel.
    chip: *Chip,
    /// Determines which one of the channels located at
    /// `/sys/class/pwm/pwmchip{chip.number}/pwm*` to access.
    number: u8,

    /// Containes value of period which was lastly set or get.
    period_ns_cached: ?u64 = null,
    /// **This field should not be accessed manualy. See `Channel.file`.**
    ///
    /// Containes file handles to already opened files under this channel. The
    /// handles are reused to speed up the access.
    files_opened: FilesOpened = .{},

    /// Initializes and exports the channel.
    ///
    /// * `chip` - the parent chip of this channel.
    /// * `channel_nr` - determines which one of the channels located at
    ///   `/sys/class/pwm/pwmchip{chip.number}/pwm*` to access.
    fn init(chip: *Chip, channel_nr: u8) !Channel {
        var channel = Channel{ .chip = chip, .number = channel_nr };
        try channel.export_();
        return channel;
    }

    /// Exports this channel, only if not already exported.
    fn export_(self: *Channel) !void {
        if (try self.isExported()) return;
        try self.chip.export_(self.number);
    }

    /// Deinitializes the channel.
    pub fn deinit(self: *Channel) void {
        self.unexport() catch |err| std.debug.panic(
            "couldn't unexport PWM channel {} on chip {}: {}\n",
            .{ self.number, self.chip.number, err },
        );

        inline for (std.meta.fields(FilesOpened)) |f| {
            const maybe_file = @field(self.files_opened, f.name);
            if (maybe_file) |file_| file_.close();
        }
        self.chip.channels[self.number] = null;
    }

    /// Unexports this channel, only if already exported.
    fn unexport(self: *Channel) !void {
        if (!try self.isExported()) return;
        try self.chip.unexport(self.number);
    }

    /// Checks if the channel is already exported, i.e. if directory
    /// `/sys/class/pwm/pwmchip{self.chip.number}/pwm{self.number}` exists.
    fn isExported(self: *const Channel) !bool {
        var buffer: PathPwmBuffer = undefined;
        const root_path = try self.path(&buffer, "", .{});
        return doesPathExist(root_path);
    }

    /// Sets both period and duty cycle in a user-friendly manner.
    ///
    /// * `params.frequency` - requested frequency in Hz (must be non-zero);
    ///   pass null to skip setting it and reuse most recently set frequency.
    /// * `params.duty_cycle_ratio` - part of the PWM period, that the wave
    ///   should be set to high (in range [0.0, 1.0]).
    pub fn setParameters(self: *Channel, params: struct {
        frequency: ?u64,
        duty_cycle_ratio: f32,
    }) !void {
        if (params.frequency == 0) return error.PwmFrequencyZero;

        const period_ns = if (params.frequency) |frequency|
            std.time.ns_per_s / frequency
        else
            self.period_ns_cached orelse return error.FirstUseFrequencyNull;
        const duty_cycle_ns = @as(f32, @floatFromInt(period_ns)) * params.duty_cycle_ratio;

        if (params.frequency != null) try self.setPeriodNs(period_ns);
        try self.setDutyCycleNs(@intFromFloat(duty_cycle_ns));
    }

    /// Sets the channel's period in nanoseconds.
    pub fn setPeriodNs(self: *Channel, period_ns: u64) !void {
        const file_ = try self.file(.period, .{ .mode = .read_write });
        try file_.writer().print("{}", .{period_ns});
        self.period_ns_cached = period_ns;
    }

    /// Gets the channel's period in nanoseconds.
    pub fn getPeriodNs(self: *Channel) !u64 {
        const file_ = try self.file(.period, .{ .mode = .read_write });
        try file_.seekTo(0);
        const period_ns = try readInt(u64, 16, file_, 10);
        self.period_ns_cached = period_ns;
        return period_ns;
    }

    /// Sets the channel's duty cycle in nanoseconds.
    /// ---
    /// **Note:** can only be called after setting a frequency/period.
    pub fn setDutyCycleNs(self: *Channel, duty_cycle_ns: u64) !void {
        const file_ = try self.file(.duty_cycle, .{ .mode = .read_write });
        try file_.writer().print("{}", .{duty_cycle_ns});
    }

    /// Gets the channel's duty cycle in nanoseconds.
    pub fn getDutyCycleNs(self: *Channel) !u64 {
        const file_ = try self.file(.duty_cycle, .{ .mode = .read_write });
        try file_.seekTo(0);
        return try readInt(u64, 16, file_, 10);
    }

    /// Sets the channel's polarity.
    /// ---
    /// **Note:** can only be changed after setting a frequency/period and when
    /// the chip is disabled.
    pub fn setPolarity(self: *Channel, value: Polarity) !void {
        if (try self.isEnabled()) return error.Enabled;

        const file_ = try self.file(.polarity, .{ .mode = .read_write });
        try file_.writer().writeAll(value.serialize());
    }

    /// Gets the channel's polarity.
    pub fn getPolarity(self: *Channel) !Polarity {
        const file_ = try self.file(.polarity, .{ .mode = .read_write });
        try file_.seekTo(0);

        var buffer: [16]u8 = undefined;
        const bytes_read = try file_.readAll(&buffer);
        if (bytes_read == buffer.len) return error.BufferTooSmall;

        var tokens = std.mem.tokenizeAny(u8, buffer[0..bytes_read], &WHITESPACE);
        const first_token = tokens.next() orelse return error.Empty;
        return try Polarity.parse(first_token);
    }

    /// Enables the channel.
    /// ---
    /// **Note:** can only be called after setting a frequency/period.
    pub fn enable(self: *Channel) !void {
        try self.setEnable(true);
    }

    /// Disables the channel.
    /// ---
    /// **Note:** can only be called after setting a frequency/period.
    pub fn disable(self: *Channel) !void {
        try self.setEnable(false);
    }

    /// Enables or disables the channel.
    /// ---
    /// **Note:** can only be called after setting a frequency/period.
    fn setEnable(self: *Channel, value: bool) !void {
        const file_ = try self.file(.enable, .{ .mode = .read_write });
        _ = try file_.writer().print("{}", .{@intFromBool(value)});
    }

    /// Checks if the channel is enabled.
    pub fn isEnabled(self: *Channel) !bool {
        const file_ = try self.file(.enable, .{ .mode = .read_write });
        try file_.seekTo(0);
        return try readInt(u1, 8, file_, 10) == 1;
    }

    /// Opens file corresponding to the `location` and returns its handle. If
    /// the file has been already accessed before, the old handle is returned
    /// without opening the file for the second time.
    ///
    /// * `location` - which of the chip's file to open (see
    ///   `Chip.FilesOpened`).
    fn file(
        self: *Channel,
        comptime location: std.meta.FieldEnum(FilesOpened),
        flags: std.fs.File.OpenFlags,
    ) !std.fs.File {
        const name = @tagName(location);
        if (@field(self.files_opened, name)) |file_| return file_;

        var buffer: PathPwmBuffer = undefined;
        const path_ = try self.path(&buffer, "/" ++ name, .{});
        const file_ = try std.fs.openFileAbsolute(path_, flags);
        @field(self.files_opened, name) = file_;
        return file_;
    }

    /// Prints the path in format:
    /// `/sys/class/pwm/pwmchip{self.chip.number}/pwm{self.number}{fmt}` into the
    /// `buffer`.
    fn path(
        self: *const Channel,
        buffer: []u8,
        comptime fmt: []const u8,
        args: anytype,
    ) std.fmt.BufPrintError![]u8 {
        return try self.chip.path(
            buffer,
            "/pwm{}" ++ fmt,
            .{self.number} ++ args,
        );
    }
};

/// Channel's polarity.
pub const Polarity = enum {
    normal,
    inversed,

    fn serialize(self: Polarity) []const u8 {
        return switch (self) {
            .normal => "normal",
            .inversed => "inversed",
        };
    }

    fn parse(token: []const u8) !Polarity {
        if (std.mem.eql(u8, token, "normal")) return .normal;
        if (std.mem.eql(u8, token, "inversed")) return .inversed;
        return error.InvalidToken;
    }
};

/// Read and parse an integer value from `file`.
///
/// * `T` - type of the integer to return.
/// * `bufsize` - how many bytes to read from `file`. If the number is longer,
///   `error.BufferTooSmall` will be returned.
/// * `file` - handle to file to read from.
/// * `base` - like in `std.fmt.parseInt`.
fn readInt(comptime T: type, comptime bufsize: u8, file: std.fs.File, base: u8) !T {
    var buffer: [bufsize:0]u8 = undefined;
    const bytes_read = try file.readAll(&buffer);
    if (bytes_read == bufsize) return error.BufferTooSmall;

    var tokens = std.mem.tokenizeAny(u8, buffer[0..bytes_read], &WHITESPACE);
    const first_token = tokens.next() orelse return error.Empty;
    return try std.fmt.parseInt(T, first_token, base);
}

/// Returns the string view, but with '_' trimmed from the end.
fn stripUnderscore(str: []const u8) []const u8 {
    if (!std.mem.endsWith(u8, str, "_")) return str;
    return str[0 .. str.len - 1];
}

/// Checks if the given path exists on the system.
fn doesPathExist(path: []const u8) bool {
    return if (std.fs.accessAbsolute(path, .{})) |_| true else |_| false;
}
