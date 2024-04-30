# BunyipsLib Changelog
## v1.0.0
Initial release and first semantic version of BunyipsLib.
### Breaking changes
- Previous versions of BunyipsLib were not following semantic versioning. This version is the first to do so.
- RoadRunner methods `addNewTrajectory(...)`, `newTrajectory(...)`, and `addTrajectory()`, have been removed and replaced with `makeTrajectory()`.
  - The new method is more flexible and culminates all task creation into one method.
  - To create new trajectories for ABOM, simply use `makeTrajectory()` and call `addTask()` to represent what `build()` used to do
  - Alternatively, you can call `buildTask()` to return a task object without adding it to the trajectory.
  - To add trajectories as tasks, you may use `makeTrajectory().runSequence()`, allowing you to have access to task properties before building.