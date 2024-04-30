# BunyipsLib Changelog
## v1.0.0
Initial release and first semantic version of BunyipsLib.<br><br>
*BunyipsLib is a comprehensive robotics library tailored for FTC teams, providing a suite of development tools for efficient robot code creation, including modules for error handling, dynamic device initialization, and integrated systems like RoadRunner, FtcDashboard, Vision, Command-based paradigms, and more. Designed for ease of use and versatility, BunyipsLib aims to streamline the programming process for students of all skill levels, fostering rapid development and code reusability across different robots.*
### Breaking changes
- Previous versions of BunyipsLib were not following semantic versioning. This version is the first to do so.
- RoadRunner methods `addNewTrajectory(...)`, `newTrajectory(...)`, and `addTrajectory()`, have been removed and replaced with `makeTrajectory()`.
  - The new method is more flexible and culminates all task and trajectory creation into one method.
  - To create new trajectories for ABOM, simply use `makeTrajectory()` and call `addTask()` to represent what `build()` used to do
  - Alternatively, you can call `buildTask()` to return a task object without adding it to the task list (for grouping or manual allocation).
  - To add pre-built trajectories as tasks, you may use `makeTrajectory().runSequence()`, allowing you to have access to task properties before building.
