# Host tests

Run `sh tests/run.sh` with a C11 compiler supporting AddressSanitizer and UBSan.
Use `CC` and `SDKROOT` to select a compatible compiler/SDK when needed.
The test compiles the actual control implementation with simulated GPIO, PWM
and RTOS services. It checks independent duties, opposite directions, safe
reversal, mapping/inversion, zero/sleep and invalid-command rejection.

ULT cases include: existing low at creation, wake pulse, continued commands
while latched/low, one queued event/callback until acknowledgement, immutable
capture counter, no worker output writes, pending-event clear rejection,
acknowledgement from callback, clear while low without immediate retrigger,
callback-free polling, and no automatic restart or stop on acknowledgement.
Peripheral write failure still attempts a stop, without creating a ULT signal.
Context tests cover initialization, wake, motor start without sleep, partial/full
coast, reversal, the inclusive trailing-window boundary, zero window,
unassociated steady drive, repeated zero without window extension, ordinary
duty changes, polling snapshots, and delayed callbacks retaining capture time
and context despite a later transition.

The mock dispatches the real worker deterministically, without a concurrent
scheduler. Mutex contention, real task lifetime, cross-core races, ISR timing,
the actual PWM backends and electrical behavior are not validated by this
host test. Build both backends with ESP-IDF and perform the hardware tests in
the example's AGENTS-APP.md before claiming hardware validation.
