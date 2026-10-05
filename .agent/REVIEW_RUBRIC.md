# Review Rubric

A milestone FAILS if any of the following occurs:

- required functionality is removed
- frontend uses fake robot state
- ROS bridge is bypassed
- command arbiter is bypassed
- safety validation is weakened
- map revision checks are weakened
- tests are deleted or weakened merely to pass
- runtime evidence is missing
- `/` does not restore the WareTwin Overview using active map and backend/ROS runtime data
- a robot marker is fabricated, mismatched to the map revision, or has no fresh TF pose
- removed product APIs remain routed
- external schedules/orders can issue NAVIGATE
- LiDAR overlay geometry is not transformed using the scan's exact timestamped TF
- MAP POINT or TAG goal bypasses a current validated preview/map identity check
- build fails
- tests fail
- ROS runtime is not verified when required

Reviewer must report:

- PASS or FAIL
- blocking issues
- exact files/functions involved
- required fix
- regression test required
- runtime acceptance evidence and any unverified/blocked items

Branch safety is mandatory: all edits and commits belong to `robot-real-sim`;
`web-simulation` is read-only and must remain unchanged.
