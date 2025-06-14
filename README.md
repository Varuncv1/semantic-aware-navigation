# 🤖 Semantic-Aware Multi-Modal Navigation

This project implements a **semantic-aware navigation system** for mobile robots, combining RGB-D and LiDAR perception, semantic segmentation, SLAM, reinforcement learning, and intent prediction. The goal is to create a robot that can navigate not only by geometry but also by understanding *what* is around it (e.g., people, doors, furniture) and adapting accordingly.

---

## 🧠 Project Overview

Traditional robots navigate based on geometry — where walls and obstacles are. But humans move with context — we avoid people, go through open doors, approach chairs. This project equips a mobile robot with the **semantic understanding** and **adaptive navigation behavior** required for real-world environments.

### 🧩 Key Features

* **RGB-D and LiDAR Fusion** for 3D perception
* **Semantic Segmentation** using DeepLabV3+
* **Semantic SLAM** with RTAB-Map and label fusion
* **Reinforcement Learning** for intelligent navigation
* **Intent Prediction** to model human motion
* **Simulation in Gazebo** and potential real-world deployment

---

## 📂 Project Structure

```
semantic-aware-navigation/
│
├── perception/         # Sensor fusion and segmentation
├── mapping/            # SLAM and semantic map construction
├── navigation/         # RL policy and planners
├── models/             # DL models: segmentation, RL, intent
├── ros/                # ROS packages, launch files
├── simulation/         # Gazebo world and test scripts
├── utils/              # Logging, visualization, and helpers
├── config/             # Config files (sensors, models, env)
├── requirements.txt    # Python dependencies
├── setup.py            # Optional Python setup
├── README.md           # This file
└── .pre-commit-config.yaml  # Git hook for formatting/linting
```

---

## 🛠️ Setup Instructions

### 1. Clone the Repository

```bash
git clone https://github.com/Varuncv1/semantic-aware-navigation.git
cd semantic-aware-navigation
```

### 2. Create a Virtual Environment

```bash
python -m venv venv
source venv/bin/activate
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

### 4. Build ROS Workspace

```bash
cd ros
catkin_make   # or colcon build
source devel/setup.bash
```

### 5. Install Pre-commit Hooks

```bash
pip install pre-commit
pre-commit install
```

---

## 🚀 Execution Steps

### Run Semantic Segmentation & Projection

```bash
python perception/segment_and_project.py
```

### Launch Semantic SLAM (ROS)

```bash
roslaunch semantic_nav semantic_nav.launch
```

### Train & Run RL Policy

```bash
python models/rl_policy/train.py   # Training
python navigation/rl_controller.py # Inference
```

### Simulate in Gazebo

```bash
./simulation/launch_sim.sh
```

---

## 📊 Example Use Case

**Input:** RGB + depth from RealSense
**Output:** 3D semantic point cloud
**Map:** Fused semantic SLAM map
**Behavior:** Robot navigates toward known targets while avoiding dynamic or sensitive classes (e.g., people, glass)

---

## 🧪 Testing & Debugging

* Visualize point clouds using Open3D
* Inspect ROS topics with `rqt_graph` and `rosbag`
* Run simulation tests:

  ```bash
  python simulation/test_scenarios/test_semantic_nav.py
  ```

---

## 🤝 Contributing

Contributions are welcome!

```bash
git checkout -b feature/my-feature
git commit -m "Add my feature"
git push origin feature/my-feature
```

Open a PR and we'll review.

---

## 👤 Author

Varun CV
GitHub: [@Varuncv1](https://github.com/Varuncv1)
Email: [varuncv2001@gmail.com](mailto:varuncv2001@gmail.com)
