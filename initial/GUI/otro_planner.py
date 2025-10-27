# otro_planner.py
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict
from PyQt5 import QtCore, QtGui, QtWidgets
import math, heapq

# === APF continuo (tu función) ===
from super_planner import apf_search

Point = QtCore.QPointF


# ========================= A* CONFIG =========================
@dataclass
class PlannerParams:
    grid: int = 20               # tamaño de celda (px)
    weight: float = 1.0          # w=1 -> A*, w>1 -> más "vago"
    diagonals: bool = True
    safety_margin: int = 2       # px extra al radio del robot
    viz_delay_ms: int = 1        # retardo de visualización (ms)


class AStarPlanner(QtCore.QObject):
    """Planifica sobre rejilla y respeta el radio del robot."""
    progressed = QtCore.pyqtSignal(int)
    explored_point = QtCore.pyqtSignal(Point)
    path_ready = QtCore.pyqtSignal(list)

    def __init__(self, mapview, params: PlannerParams):
        super().__init__()
        self.view = mapview
        self.params = params
        self._abort = False

    def abort(self):
        self._abort = True

    # --- util de geometría ---
    def _dist(self, a: Tuple[int,int], b: Tuple[int,int]) -> float:
        return math.hypot(a[0]-b[0], a[1]-b[1])

    def _cell_center(self, i: int, j: int) -> Point:
        g = self.params.grid
        return Point((i + 0.5) * g, (j + 0.5) * g)

    def _to_cell(self, p: Point) -> Tuple[int,int]:
        g = self.params.grid
        return int(p.x() // g), int(p.y() // g)

    def _grid_size(self) -> Tuple[int,int]:
        r = self.view.scene.sceneRect()
        return max(1, int(r.width() // self.params.grid)), max(1, int(r.height() // self.params.grid))

    def _is_free_cell(self, i: int, j: int, inflate: float) -> bool:
        cols, rows = self._grid_size()
        if i < 0 or j < 0 or i >= cols or j >= rows:
            return False
        c = self._cell_center(i, j)
        rect = self.view.scene.sceneRect()
        if (c.x() - inflate) < rect.left() or (c.x() + inflate) > rect.right() or \
           (c.y() - inflate) < rect.top()  or (c.y() + inflate) > rect.bottom():
            return False
        for it in self.view.obstacle_items:
            br = it.sceneBoundingRect()
            oc = br.center()
            orad = br.width()/2.0
            if QtCore.QLineF(c, oc).length() <= (orad + inflate):
                return False
        return True

    def plan(self):
        """Ejecuta A* emitiendo señales para visualizar."""
        self._abort = False

        r_robot = float(getattr(self.view.robot, "_r", 12))
        inflate = r_robot + float(self.params.safety_margin)

        start_c = self._to_cell(self.view.robot.center())
        goal_c  = self._to_cell(self.view.flag.sceneBoundingRect().center())

        def nearest_free(base):
            cols, rows = self._grid_size()
            best = None; bestd=1e9
            for i in range(cols):
                for j in range(rows):
                    if self._is_free_cell(i, j, inflate):
                        d = self._dist((i,j), base)
                        if d < bestd:
                            bestd = d; best = (i,j)
            return best

        if not self._is_free_cell(*start_c, inflate):
            start_c = nearest_free(start_c)
        if not self._is_free_cell(*goal_c, inflate):
            goal_c = nearest_free(goal_c)

        if start_c is None or goal_c is None:
            self.path_ready.emit([])
            return

        neigh = [(-1,0),(1,0),(0,-1),(0,1)] + ([(-1,-1),(-1,1),(1,-1),(1,1)] if self.params.diagonals else [])

        def h(a,b):
            return self.params.weight * self._dist(a,b)

        openq = []
        heapq.heappush(openq, (0+h(start_c,goal_c), 0.0, start_c))
        came: Dict[Tuple[int,int], Optional[Tuple[int,int]]] = {start_c: None}
        gs: Dict[Tuple[int,int], float] = {start_c: 0.0}
        expanded = 0

        while openq and not self._abort:
            _, gcost, cur = heapq.heappop(openq)
            expanded += 1

            self.explored_point.emit(self._cell_center(*cur))
            if expanded % 200 == 0:
                self.progressed.emit(expanded)

            if cur == goal_c:
                path_cells = []
                node = cur
                while node is not None:
                    path_cells.append(node)
                    node = came[node]
                path_cells.reverse()
                coords = [self._cell_center(i,j) for (i,j) in path_cells]
                self.path_ready.emit(coords)
                return

            for dx,dy in neigh:
                nx, ny = cur[0]+dx, cur[1]+dy
                step = math.hypot(dx,dy)
                if step <= 0:
                    continue
                if not self._is_free_cell(nx, ny, inflate):
                    continue
                cand_g = gcost + step
                if (nx,ny) not in gs or cand_g < gs[(nx,ny)]:
                    gs[(nx,ny)] = cand_g
                    prio = cand_g + h((nx,ny), goal_c)
                    heapq.heappush(openq, (prio, cand_g, (nx,ny)))
                    came[(nx,ny)] = cur

        self.path_ready.emit([])


# ========================= APF ADAPTER =========================
class SuperPlannerAdapter(QtCore.QObject):
    """Adaptador para apf_search con mismas señales que AStarPlanner."""
    progressed = QtCore.pyqtSignal(int)
    explored_point = QtCore.pyqtSignal(Point)
    path_ready = QtCore.pyqtSignal(list)

    def __init__(self, mapview):
        super().__init__()
        self.view = mapview
        # Parámetros por defecto APF
        self.max_iters = 20000
        self.influence_area = 80.0
        self.attraction_factor = 1.0
        self.repulsion_factor = 20.0
        self.close_enough = 1.0

    def abort(self):
        pass  # el APF aquí es de un solo disparo

    def plan(self):
        rc = self.view.robot.center()
        start = (rc.x(), rc.y())
        gc = self.view.flag.sceneBoundingRect().center()
        goal = (gc.x(), gc.y())
        obstacles = []
        for it in self.view.obstacle_items:
            br = it.sceneBoundingRect()
            oc = br.center()
            rad = br.width() / 2.0
            obstacles.append((oc.x(), oc.y(), rad))

        traj = apf_search(
            start_xy=start,
            goal_xy=goal,
            obstacles=obstacles,
            max_iters=self.max_iters,
            influence_area=self.influence_area,
            attraction_factor=self.attraction_factor,
            repulsion_factor=self.repulsion_factor,
            close_enough=self.close_enough
        )

        coords = [Point(float(x), float(y)) for (x, y) in traj.tolist()]
        self.path_ready.emit(coords)


# ========================= PANEL UI =========================
class PlannerPanel(QtWidgets.QFrame):
    """
    Panel de la derecha: parámetros + visualización y control de ejecución.
    Se engancha al botón Play externo (llamar a on_play()).
    """
    def __init__(self, mapview, info_cb):
        super().__init__()
        self.view = mapview
        self.info_cb = info_cb

        self.setObjectName("planner_panel")
        self.setStyleSheet("QFrame#planner_panel{border:2px dashed #bbb;border-radius:14px;background:#fcfcfc;}")

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(14,14,14,14)
        lay.setSpacing(10)

        # Título (se actualiza con el selector)
        self._title = QtWidgets.QLabel("Planner A*")
        f = self._title.font(); f.setPointSize(14); f.setBold(True); self._title.setFont(f)
        lay.addWidget(self._title)

        # Selector de algoritmo
        self.cb_algo = QtWidgets.QComboBox()
        self.cb_algo.addItems(["A* (grid)", "APF (super_planner)"])
        self.cb_algo.currentIndexChanged.connect(self._on_algo_changed)
        lay.addWidget(self.cb_algo)

        # ----- BLOQUE A*: parámetros -----
        self.gb_astar = QtWidgets.QGroupBox("Parámetros A*")
        lay_astar = QtWidgets.QVBoxLayout(self.gb_astar)

        self.sb_grid = QtWidgets.QSpinBox(); self.sb_grid.setRange(6,120); self.sb_grid.setValue(20)
        self._row(lay_astar,"Tamaño celda (px):", self.sb_grid)

        self.dsb_w   = QtWidgets.QDoubleSpinBox(); self.dsb_w.setRange(1.0,5.0); self.dsb_w.setSingleStep(0.1); self.dsb_w.setValue(1.0)
        self._row(lay_astar,"Peso heurístico (w):", self.dsb_w)

        self.cb_diag = QtWidgets.QCheckBox("Permitir diagonales"); self.cb_diag.setChecked(True)
        lay_astar.addWidget(self.cb_diag)

        self.sb_margin = QtWidgets.QSpinBox(); self.sb_margin.setRange(0,30); self.sb_margin.setValue(2)
        self._row(lay_astar, "Margen seguridad (px):", self.sb_margin)

        self.sb_delay = QtWidgets.QSpinBox(); self.sb_delay.setRange(0,20); self.sb_delay.setValue(1)
        self._row(lay_astar, "Retardo visualización (ms):", self.sb_delay)

        lay.addWidget(self.gb_astar)

        # ----- BLOQUE APF: parámetros -----
        self.gb_apf = QtWidgets.QGroupBox("Parámetros APF")
        lay_apf = QtWidgets.QVBoxLayout(self.gb_apf)

        self.sb_apf_iters = QtWidgets.QSpinBox(); self.sb_apf_iters.setRange(1, 300000); self.sb_apf_iters.setValue(20000)
        self._row(lay_apf, "Iteraciones máx.:", self.sb_apf_iters)

        self.sb_apf_d0 = QtWidgets.QSpinBox(); self.sb_apf_d0.setRange(1, 500); self.sb_apf_d0.setValue(80)
        self._row(lay_apf, "Área de influencia (d0):", self.sb_apf_d0)

        self.dsb_apf_att = QtWidgets.QDoubleSpinBox(); self.dsb_apf_att.setDecimals(3); self.dsb_apf_att.setRange(0.001, 1000.0); self.dsb_apf_att.setValue(1.0)
        self._row(lay_apf, "Factor de atracción:", self.dsb_apf_att)

        self.dsb_apf_rep = QtWidgets.QDoubleSpinBox(); self.dsb_apf_rep.setDecimals(3); self.dsb_apf_rep.setRange(0.001, 5000.0); self.dsb_apf_rep.setValue(20.0)
        self._row(lay_apf, "Factor de repulsión:", self.dsb_apf_rep)

        self.dsb_apf_tol = QtWidgets.QDoubleSpinBox(); self.dsb_apf_tol.setDecimals(2); self.dsb_apf_tol.setRange(0.01, 50.0); self.dsb_apf_tol.setValue(1.00)
        self._row(lay_apf, "Tolerancia a meta:", self.dsb_apf_tol)

        lay.addWidget(self.gb_apf)

        # Estado inicial de visibilidad
        self._on_algo_changed(self.cb_algo.currentIndex())

        # Barra de progreso
        self.pb_status = QtWidgets.QProgressBar(); self.pb_status.setRange(0,0); self.pb_status.setVisible(False)
        lay.addWidget(self.pb_status)

        # Leyenda
        self.legend = QtWidgets.QLabel("Exploración: gris •  Ruta: azul")
        lay.addWidget(self.legend)
        lay.addStretch(1)

        # Capas gráficas
        self._explored_layer = QtWidgets.QGraphicsItemGroup()
        self._path_item = QtWidgets.QGraphicsPathItem()
        self._path_item.setPen(QtGui.QPen(QtCore.Qt.blue, 4, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap, QtCore.Qt.RoundJoin))
        self._path_item.setZValue(-1)
        self.view.scene.addItem(self._path_item)
        self.view.scene.addItem(self._explored_layer)

        # Animación
        self.anim_timer = QtCore.QTimer(self)
        self.anim_timer.timeout.connect(self._advance_robot)
        self._anim_path: List[Point] = []
        self._anim_index = 0

        self._running = False
        self._planner: Optional[QtCore.QObject] = None

    # ----- helpers UI -----
    def _row(self, lay, text, widget):
        h = QtWidgets.QHBoxLayout()
        lab = QtWidgets.QLabel(text)
        h.addWidget(lab,1)
        h.addWidget(widget,0)
        lay.addLayout(h)

    def _on_algo_changed(self, idx: int):
        use_astar = self.cb_algo.currentText().startswith("A*")
        self.gb_astar.setVisible(use_astar)
        self.gb_apf.setVisible(not use_astar)
        self._title.setText("Planner A*" if use_astar else "Planner APF")

    # --------- API externa ----------
    def on_play(self):
        if self._running:
            self.stop()
            return
        self.start()

    def start(self):
        if self._running:
            self.stop()

        self._set_scene_interactive(False)
        self.anim_timer.stop()
        self.clear_graphics()
        self._running = True
        self.pb_status.setVisible(True)

        algo = self.cb_algo.currentText()
        if algo.startswith("A*"):
            params = PlannerParams(
                grid=self.sb_grid.value(),
                weight=self.dsb_w.value(),
                diagonals=self.cb_diag.isChecked(),
                safety_margin=self.sb_margin.value(),
                viz_delay_ms=self.sb_delay.value()
            )
            self._planner = AStarPlanner(self.view, params)
            self._planner.progressed.connect(lambda n: self.info_cb(f"Explorados: {n}"))
            self._planner.explored_point.connect(self._draw_explored)
            self._planner.path_ready.connect(self._on_path_ready)
            QtCore.QTimer.singleShot(0, self._planner.plan)
        else:
            self._planner = SuperPlannerAdapter(self.view)
            # Mapear parámetros APF desde el panel
            self._planner.max_iters = self.sb_apf_iters.value()
            self._planner.influence_area = self.sb_apf_d0.value()
            self._planner.attraction_factor = self.dsb_apf_att.value()
            self._planner.repulsion_factor = self.dsb_apf_rep.value()
            self._planner.close_enough = self.dsb_apf_tol.value()
            self._planner.path_ready.connect(self._on_path_ready)
            QtCore.QTimer.singleShot(0, self._planner.plan)

    def stop(self):
        if self._planner and hasattr(self._planner, "abort"):
            self._planner.abort()
        self.anim_timer.stop()
        self._running = False
        self.pb_status.setVisible(False)
        self._set_scene_interactive(True)

    def clear_graphics(self):
        for it in list(self._explored_layer.childItems()):
            self._explored_layer.removeFromGroup(it)
            self.view.scene.removeItem(it)
        self._path_item.setPath(QtGui.QPainterPath())
        self._anim_path = []
        self._anim_index = 0

    # --------- visualización ----------
    def _draw_explored(self, p: Point):
        dot = QtWidgets.QGraphicsEllipseItem(0,0,6,6)
        dot.setBrush(QtGui.QBrush(QtGui.QColor(120,120,120,130)))
        dot.setPen(QtGui.QPen(QtCore.Qt.NoPen))
        dot.setPos(p.x()-3, p.y()-3)
        self._explored_layer.addToGroup(dot)
        d = self.sb_delay.value()
        if d > 0:
            loop = QtCore.QEventLoop()
            QtCore.QTimer.singleShot(d, loop.quit); loop.exec_()

    def _on_path_ready(self, coords: List[Point]):
        self.pb_status.setVisible(False)
        if not coords:
            self.info_cb("No se encontró camino.")
            self._set_scene_interactive(True)
            self._running = False
            return

        start = coords[0]
        self.view.robot.setPos(start.x() - self.view.robot.rect().width()/2,
                               start.y() - self.view.robot.rect().height()/2)

        path = QtGui.QPainterPath()
        path.moveTo(coords[0])
        for pt in coords[1:]:
            path.lineTo(pt)
        self._path_item.setPath(path)

        self._anim_path = coords
        self._anim_index = 0
        self.anim_timer.start(12)

    def _advance_robot(self):
        if self._anim_index >= len(self._anim_path):
            self.anim_timer.stop()
            self._set_scene_interactive(True)
            self._running = False
            self.info_cb("Ruta completada.")
            return

        target = self._anim_path[self._anim_index]
        rc = self.view.robot.center()
        v = QtCore.QPointF(target - rc)
        dist = math.hypot(v.x(), v.y())
        step = 3.5
        if dist <= step:
            self.view.robot.setPos(target.x() - self.view.robot.rect().width()/2,
                                   target.y() - self.view.robot.rect().height()/2)
            self._anim_index += 1
            if self._anim_index > 1:
                cut_path = QtGui.QPainterPath()
                cut_path.moveTo(self._anim_path[self._anim_index-1])
                for pt in self._anim_path[self._anim_index:]:
                    cut_path.lineTo(pt)
                self._path_item.setPath(cut_path)
            return

        nx = rc.x() + step * (v.x()/dist)
        ny = rc.y() + step * (v.y()/dist)
        self.view.robot.setPos(nx - self.view.robot.rect().width()/2,
                               ny - self.view.robot.rect().height()/2)

    def _set_scene_interactive(self, enabled: bool):
        self.view.setAcceptDrops(enabled)
        self.view.flag.setFlag(QtWidgets.QGraphicsItem.ItemIsMovable, enabled)
        for it in self.view.obstacle_items:
            it.setFlag(QtWidgets.QGraphicsItem.ItemIsMovable, enabled)
        self.info_cb("Listo." if enabled else "Planificando…")
