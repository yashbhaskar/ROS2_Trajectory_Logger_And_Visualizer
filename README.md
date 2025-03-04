# 🚀ROS2 Trajectory Logger & Visualizer📍
This ROS2 package enables real-time trajectory collection, storage, and visualization for mobile robots. It consists of two key nodes:

✅ Trajectory Publisher and Saver Node
- 📝 Collects and publishes the robot's trajectory as a marker array.
- 💾 Provides a ROS service to save trajectory data in JSON, CSV, or YAML formats.
- ⏳ Allows users to specify a time duration for saving recent trajectory data.

✅ Trajectory Reader and Publisher Node
- 📂 Reads saved trajectory files.
- 🔄 Transforms the trajectory to the odom frame.
- 📡 Publishes the transformed trajectory for visualization.

This package is ideal for path tracking, debugging, and performance analysis in robotics applications using ROS2. 🤖✨
