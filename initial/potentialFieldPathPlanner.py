import sys
from PyQt5.QtCore import Qt, QPointF, QRectF
from PyQt5.QtGui import QBrush, QPen, QPainterPath
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QFormLayout,
    QPushButton, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem, QGraphicsPathItem,
    QSpinBox, QDoubleSpinBox, QLabel, QGroupBox, QMessageBox
)

# --- Backend (se importa desde tu proyecto) ---
sys.path.append("./..")
from obstacle import Obstacle
from robot import Robot
from potentialFieldPathPlanner import PotentialFieldPathPlanner

# ------------------ Items gráficos ------------------
class CircleItem(QGraphicsEllipseItem):
    def __init__(self, x, y, r, color=Qt.darkGray, filled=True):
        super().__init__(0, 0, 2*r, 2*r)
        self.setPos(x - r, y - r)  # posición = esquina sup-izq del bounding
        pen = QPen(color)
        pen.setWidth(2)
        self.setPen(pen)
        if filled:
            self.setBrush(QBrush(color))
        self.r = r
        self.setFlag(QGraphicsEllipseItem.ItemIsMovable, True)
        self.setFlag(QGraphicsEllipseItem.ItemSendsScenePositionChanges, True)

    def center(self) -> QPointF:
        rect = self.rect()
        return self.pos() + QPointF(rect.width()/2, rect.height()/2)

    def set_center(self, x, y):
        self.setPos(x - self.r, y - self.r)


class PathItem(QGraphicsPathItem):
    def __init__(self):
        super().__init__()
        pen = QPen(Qt.blue)
        pen.setWidth(2)
        self.setPen(pen)

    def set_points(self, points):
        if not len(points):
            self.setPath(QPainterPath())
            return
        p = QPainterPath(QPointF(points[0][0], points[0][1]))
        for x, y in points[1:]:
            p.lineTo(x, y)
        self.setPath(p)


# ------------------ Vista del mundo ------------------
class PlannerView(QGraphicsView):
    MODE_SELECT = 0
    MODE_SET_START = 1
    MODE_SET_GOAL = 2
    MODE_ADD_OBS = 3

    def __init__(self, scene_rect=QRectF(0, 0, 600, 600)):
        super().__init__()
        self.setRenderHint(self.paintEngine().Painter.Antialiasing, True)
        self.scene = QGraphicsScene(scene_rect)
        self.setScene(self.scene)
        self.setBackgroundBrush(QBrush(Qt.white))
        self.mode = self.MODE_SELECT

        # elementos
        self.robot_item = CircleItem(50, 50, 10, Qt.darkGreen)
        self.robot_item.setFlag(QGraphicsEllipseItem.ItemIsMovable, True)
        self.scene.addItem(self.robot_item)

        self.start_item = CircleItem(50, 50, 4, Qt.green)
        self.start_item.setFlag(QGraphicsEllipseItem.ItemIsMovable, True)
        self.scene.addItem(self.start_item)

        self.goal_item = CircleItem(550, 550, 6, Qt.red)
        self.goal_item.setFlag(QGraphicsEllipseItem.ItemIsMovable, True)
        self.scene.addItem(self.goal_item)

        self.path_item = PathItem()
        self.scene.addItem(self.path_item)

        self.obstacle_items = []

    # --- helpers ---
    def clear_path(self):
        self.path_item.set_points([])

    def add_obstacle_circle(self, x, y, r=15):
        item = CircleItem(x, y, r, Qt.darkGray)
        self.scene.addItem(item)
        self.obstacle_items.append(item)
        return item

    # --- modos con ratón ---
    def set_mode(self, mode):
        self.mode = mode
        self.clear_path()

    def mousePressEvent(self, event):
        pos = self.mapToScene(event.pos())
        if self.mode == self.MODE_SET_START:
            self.start_item.set_center(pos.x(), pos.y())
        elif self.mode == self.MODE_SET_GOAL:
            self.goal_item.set_center(pos.x(), pos.y())
        elif self.mode == self.MODE_ADD_OBS:
            self.add_obstacle_circle(pos.x(), pos.y(), r=15)
        else:
            super().mousePressEvent(event)

    # extracción de datos para el backend
    def get_robot_state(self):
        c = self.robot_item.center()
        return float(c.x()), float(c.y()), float(self.robot_item.r)

    def get_start_state(self):
        c = self.start_item.center()
        return float(c.x()), float(c.y())

    def get_goal_state(self):
        c = self.goal_item.center()
        return float(c.x()), float(c.y())

    def get_obstacles(self):
        obs = []
        for it in self.obstacle_items:
            c = it.center()
            obs.append((float(c.x()), float(c.y()), float(it.r)))
        return obs


# ------------------ Ventana principal ------------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("APF Path Planner – GUI")
        self.resize(900, 650)

        # centro: vista
        self.view = PlannerView()

        # panel de control
        controls = self._build_controls()

        root = QWidget()
        layout = QHBoxLayout(root)
        layout.addWidget(self.view, 1)
        layout.addWidget(controls)
        self.setCentralWidget(root)

        # backend (se inicializa al vuelo al pulsar RUN)
        self.planner = None

    # ------- UI -------
    def _build_controls(self) -> QWidget:
        panel = QWidget()
        v = QVBoxLayout(panel)

        # Grupo: modos
        gb_modes = QGroupBox("Edición")
        fm = QVBoxLayout(gb_modes)
        btn_select = QPushButton("Seleccionar/Mover")
        btn_start = QPushButton("Fijar START")
        btn_goal = QPushButton("Fijar GOAL")
        btn_obs = QPushButton("Añadir obstáculo")
        for b in (btn_select, btn_start, btn_goal, btn_obs):
            fm.addWidget(b)
        v.addWidget(gb_modes)

        btn_select.clicked.connect(lambda: self.view.set_mode(PlannerView.MODE_SELECT))
        btn_start.clicked.connect(lambda: self.view.set_mode(PlannerView.MODE_SET_START))
        btn_goal.clicked.connect(lambda: self.view.set_mode(PlannerView.MODE_SET_GOAL))
        btn_obs.clicked.connect(lambda: self.view.set_mode(PlannerView.MODE_ADD_OBS))

        # Grupo: parámetros
        gb_params = QGroupBox("Parámetros APF")
        fp = QFormLayout(gb_params)

        self.sp_iters = QSpinBox(); self.sp_iters.setRange(1, 200000); self.sp_iters.setValue(20000)
        self.sp_infl  = QSpinBox(); self.sp_infl.setRange(1, 500); self.sp_infl.setValue(80)
        self.sp_att   = QDoubleSpinBox(); self.sp_att.setDecimals(3); self.sp_att.setRange(0.001, 1000.0); self.sp_att.setValue(1.0)
        self.sp_rep   = QDoubleSpinBox(); self.sp_rep.setDecimals(3); self.sp_rep.setRange(0.001, 5000.0); self.sp_rep.setValue(20.0)
        self.sp_rbot  = QSpinBox(); self.sp_rbot.setRange(1, 100); self.sp_rbot.setValue(10)

        fp.addRow("Iteraciones máx.", self.sp_iters)
        fp.addRow("Área influencia (d0)", self.sp_infl)
        fp.addRow("Factor atracción", self.sp_att)
        fp.addRow("Factor repulsión", self.sp_rep)
        fp.addRow("Radio robot", self.sp_rbot)
        v.addWidget(gb_params)

        # Acciones
        btn_run = QPushButton("▶ Ejecutar planner")
        btn_clear_path = QPushButton("Borrar trayectoria")
        btn_clear_obs = QPushButton("Borrar obstáculos")
        v.addWidget(btn_run)
        v.addWidget(btn_clear_path)
        v.addWidget(btn_clear_obs)

        btn_run.clicked.connect(self.on_run)
        btn_clear_path.clicked.connect(lambda: self.view.clear_path())
        btn_clear_obs.clicked.connect(self.on_clear_obstacles)

        v.addStretch(1)
        v.addWidget(QLabel("Consejo: usa los botones de ‘Edición’ y arrastra los círculos."))
        return panel

    # ------- lógica -------
    def on_clear_obstacles(self):
        for it in self.view.obstacle_items:
            self.view.scene.removeItem(it)
        self.view.obstacle_items.clear()
        self.view.clear_path()

    def build_backend(self):
        # estados desde la vista
        sx, sy = self.view.get_start_state()
        gx, gy = self.view.get_goal_state()
        rx, ry, rr = self.view.get_robot_state()

        # Robot del backend
        robot = Robot(rx, ry, rr, "turtle")
        planner = PotentialFieldPathPlanner(robot, gx, gy)

        # parámetros
        planner.set_max_iterations(self.sp_iters.value())
        planner.set_influence_area(self.sp_infl.value())
        planner.set_attraction_factor(self.sp_att.value())
        planner.set_repulsion_factor(self.sp_rep.value())

        # obstáculos
        for x, y, r in self.view.get_obstacles():
            planner.add_obstacle(Obstacle(x, y, r))
        return planner

    def sync_robot_radius(self):
        # sincroniza el radio visual con el numérico antes de ejecutar
        _, _, rr = self.view.get_robot_state()
        new_r = self.sp_rbot.value()
        if abs(new_r - rr) > 1e-6:
            c = self.view.robot_item.center()
            self.view.scene.removeItem(self.view.robot_item)
            self.view.robot_item = CircleItem(c.x(), c.y(), new_r, Qt.darkGreen)
            self.view.scene.addItem(self.view.robot_item)

    def on_run(self):
        try:
            self.sync_robot_radius()
            self.planner = self.build_backend()
            path = self.planner.run()  # numpy array N x 2
            self.view.path_item.set_points(path.tolist())
        except Exception as e:
            QMessageBox.critical(self, "Error ejecutando el planner", str(e))


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())
