# GUI Path Planning with Artificial Potential Fields

This project implements a **Graphical User Interface (GUI)** for a simple **robot path planning system** based on the **Artificial Potential Field (APF)** method.  
It was developed as part of the course **Human-Machine Interfaces (IR2125)** in the **Robotic Intelligence** degree at *Universitat Jaume I*.

---

## 🧠 Concept

The goal of this project is to design and program a user-friendly interface that allows users to experiment with robot path planning using the **Artificial Potential Field** algorithm.  
The GUI helps visualize how attractive and repulsive forces affect the robot’s trajectory toward a goal while avoiding obstacles.

---

## 🧩 Project Structure
📂 GUIPathPlanning
├── circle.py # Defines circular geometry primitives
├── object.py # Generic object class based on Circle
├── obstacle.py # Obstacle class
├── robot.py # Robot class inheriting from Object
├── geom_utils.py # Geometric utilities (distance, unit vectors, etc.)
├── potentialFieldPathPlanner.py # Core APF planner algorithm
├── draw_utils.py # Simple matplotlib-based visualization
├── main.py # Basic PyQt5 GUI window
├── guiPathPlanning.pdf # Lab instructions and theoretical background
└── Documentación..pdf # Design documentation and brainstorming notes



---

## 🚀 How It Works

1. The **PotentialFieldPathPlanner** class computes the path from a start to a goal position:
   - The robot is attracted to the goal.
   - Obstacles generate repulsive forces to avoid collisions.
   - The resulting vector field guides the robot’s movement.

2. The **GUI (PyQt5)** allows users to:
   - Define the robot’s start and goal positions.
   - Add, remove, and resize obstacles.
   - Modify planner parameters (influence area, attraction, repulsion, iterations).
   - Visualize the resulting path interactively.

---

## 🎯 User Tasks

The GUI was designed around the following user tasks:

| ID  | Task Description |
|-----|------------------|
| **T1** | Define and edit a robot |
| **T2** | Define and edit a start position |
| **T3** | Define and edit a goal position |
| **T4** | Create, remove, and edit obstacles |
| **T5** | Set and change planner parameters |
| **T6** | Run the planner |
| **T7** | Visualize the robot, goal, obstacles, and computed path |

The most frequent tasks (T2–T7) were prioritized in the interface design, ensuring they are accessible with minimal user actions.

---

## 🧩 Design Process

The interface was created following a **user-centered design** approach:

- **Heuristic evaluation** of initial prototypes.
- **Brainstorming and individual sketches** by each team member.
- **Consolidation** of the best ideas into a single mockup.
- **Iterative refinement** during implementation.

### 🧭 Key Design Decisions
- The robot and goal are initialized automatically so users can immediately run the planner.
- A **top panel** allows adding objects and running the simulation.
- A **collapsible side panel** provides access to algorithm parameters.
- Obstacles can be **created by clicking or dragging**, and edited directly by interacting with them.
- The **Play** button and keyboard shortcut **Shift+Enter** run the simulation.
- Objects can be deleted easily using an on-hover **trash icon**.

---

## ⚙️ Installation and Execution

### Requirements
- Python 3.8+
- PyQt5
- NumPy
- Matplotlib

### Run
```bash
pip install pyqt5 numpy matplotlib
python3 main.py
