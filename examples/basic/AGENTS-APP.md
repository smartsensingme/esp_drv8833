# Basic DRV8833 example

This bench example applies 70% duty after three seconds, waits nine seconds,
coasts, waits two seconds and applies 75%. Preserve this user test sequence.
There is no periodic control/acquisition task, control-loop rate or end-to-end
deadline. The PWM peripheral generates a continuous 20 kHz carrier (50 us
period), independently of application scheduling. Do not claim equal wheel
speeds from equal duty. Encoder-based regulation belongs to a separate app.

Board defaults follow ../../docs/pins.txt for ESP32-C3 SuperMini. Motor 1
(IN1/IN2, GPIO0/1) is left, Motor 2 (GPIO2/3) is right. Only the pins are
configured in the example's menuconfig. Mapping and polarity are set in C
before creating the instance; both inversion flags are currently false.
GPIO2/8/9 are ESP32-C3 strap pins; verify attached circuits permit boot.

ULT defaults to GPIO7, EEP to GPIO10. Fault worker priority is 5 on Core 0
(also the only core of C3); the logging callback runs outside the driver lock.
ULT is notification-only: it never changes PWM/EEP or rejects commands.
The first falling edge (or existing low at create) latches one signal; further
edges update the counter without queuing callbacks until the application calls
clear_fault(). The worker copies the accepted event before callback dispatch.
Clear before dispatch returns INVALID_STATE; clear from callback is allowed.
Acknowledgement while low does not retrigger until a new falling edge occurs.
The example logs captured context and explicitly acknowledges from callback;
it does not stop motors on ULT. Context is a temporal association, not a cause.
Default context window is 2000 us after transition end, configured only in C.
No event is automatically discarded. Measure ISR/callback overhead and test
within/outside the window; manual VCC changes are unknown to the driver.
The C3 defaults no longer disable ULT; GPIO7 is enabled in the current config.
Callback delivery latency is scheduler-dependent, with no hard deadline.

Wake costs a 1 ms busy wait; a driven reversal costs up to the configured 50 us
coast interval by default. Neither occurs periodically in this example.
Keep watchdogs enabled. Hardware validation must cover PWM at 0/30/100%, both
directions, mapping/inversion, wake timing, ULT low before/during commands,
short ULT pulses, one callback per acknowledgement, continued PWM with ULT
latched/low, clear without output writes, and competing load.
Measure physical edges and stop latency externally; compilation does not prove
electrical behavior or timing bounds. Test with wheels lifted first.
