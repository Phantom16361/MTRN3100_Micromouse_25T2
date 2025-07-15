# MTRN3100-Micromouse
Repository for 2025T2 MTRN3100 Micro Mouse project

/src          
  ├── EncoderOdometry.hpp/cpp      # Stage 1
  ├── MotorController.hpp/cpp      # Stage 2
  ├── PIDController.hpp/cpp        # Stage 3 & 4
  ├── IMUManager.hpp/cpp           # Stage 5
  ├── LidarManager.hpp/cpp         # Stage 6
  ├── MotionPlanner.hpp/cpp        # Stage 7+
  ├── StateMachine.hpp/cpp         # Stage 7+
  └── utils.hpp/cpp (optional)     # For math helpers, filters, conversions, etc.

  /main_project/
    └── main.ino     <-- production maze navigation logic  # Entry point: sets up loop, state machine

  /test_velocity_pid/
    └── main.ino     <-- just runs velocity PID in loop

  /test_position_pid/
    └── main.ino     <-- drives forward set distance, logs PID loop

  /test_turning_pid/
    └── main.ino     <-- runs IMU yaw PID and logs angle

  /test_wall_follow/
    └── main.ino     <-- uses lidar PID only

/shared
  └── pin_config.hpp  <-- hardware mappings and defines
