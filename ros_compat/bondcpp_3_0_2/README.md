# Humble bondcpp 3.0.2 wait synchronization backport

Sources are from https://github.com/ros/bond_core/tree/3.0.2/bondcpp/src
and retain their upstream BSD license notices. Only `waitUntilFormed` and
`waitUntilBroken` differ: they hold the existing FSM mutex for each state read,
and release it through the existing condition variable while waiting.

The installed 3.0.2 implementation reads `sm_.getState()` without its mutex
while `bondStatusCB` can clear/transition that state. This raises smclib's
`StateUndefinedException` (`transition invoked while in transition`) from
Nav2's `createBondConnection`, even after a lifecycle node becomes ACTIVE.

This replacement is compiled only against bondcpp 3.0.2's installed headers.
It preserves the upstream library name, class layout, constructors, symbols,
timeouts and heartbeat checks. `scripts/ros_env.sh` puts this workspace's
library first for its processes. Navigation launch explicitly retains that
directory in its sanitized native Nav2 environment when the library exists;
it does not inherit unrelated overlay library paths. Verify `/proc/<pid>/maps`
for the running lifecycle manager, not only shell `ldd`. No system ROS library
is overwritten.
With another bondcpp version, CMake does not build/install this replacement.

The paired-bond C++ test runs real ROS callbacks concurrently with repeated
formation/break waits. Production two-start acceptance remains separate.
