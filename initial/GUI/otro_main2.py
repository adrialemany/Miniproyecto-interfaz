# main_topbar_gui.py (versión final: límites + dark mode + PlannerPanel + CORREGIDO v2)
import os, sys
from pathlib import Path
from PyQt5 import QtCore, QtGui, QtWidgets
from otro_planner import PlannerPanel 

# -------- Constantes de Estilo (del script 2) --------
COLOR_BG_DARKER = "#212121"
COLOR_BG_DARK = "#2b2b2b"
COLOR_BG_MEDIUM = "#3c3c3c"
COLOR_BG_LIGHT = "#4f4f4f"
COLOR_TEXT = "#f0f0f0"
COLOR_BORDER = "#555555"
COLOR_ACCENT = "#0078d4"
COLOR_ACCENT_HOVER = "#0095ff"
COLOR_ACCENT_PRESSED = "#005a9e"

DARK_MODE_STYLESHEET = f"""
    QWidget {{
        background-color: {COLOR_BG_DARK};
        color: {COLOR_TEXT};
        font-family: Inter, Segoe UI, Arial;
        font-size: 10pt;
    }}
    QMainWindow {{
        background-color: {COLOR_BG_DARKER};
    }}
    QFrame {{
        border: 1px solid {COLOR_BORDER};
        border-radius: 14px;
        background-color: {COLOR_BG_MEDIUM};
    }}
    QWidget#PlannerPanel {{
        border: 1px solid {COLOR_BORDER};
        border-radius: 14px;
        background-color: {COLOR_BG_MEDIUM};
    }}

    /* Estilos para los frames de info y añadir */
    QFrame#frame_add, QFrame#frame_info {{
         background-color: {COLOR_BG_MEDIUM};
         border: 1px solid {COLOR_BORDER};
    }}
    
    /* --- CAMBIO (CORRECCIÓN 2) --- */
    /* El frame de Play debe ser invisible */
    QFrame#frame_play {{
         background-color: transparent;
         border: none;
    }}
    /* --- FIN DEL CAMBIO --- */

    QFrame#map_frame {{
        background-color: {COLOR_BG_MEDIUM};
        border: 1px solid {COLOR_BORDER};
        border-radius: 14px;
    }}
    QLabel {{
        background-color: transparent;
        border: none;
    }}
    QPushButton {{
        background-color: {COLOR_ACCENT};
        color: {COLOR_TEXT};
        border: none;
        padding: 8px 12px;
        border-radius: 14px;
        font-weight: bold;
    }}
    QPushButton:hover {{
        background-color: {COLOR_ACCENT_HOVER};
    }}
    QPushButton:pressed {{
        background-color: {COLOR_ACCENT_PRESSED};
    }}
    QGraphicsView {{
        border: none;
        border-radius: 10px; 
        background-color: {COLOR_BG_DARK};
    }}
    /* ... ScrollBars (sin cambios) ... */
    QScrollBar:vertical {{
        background: {COLOR_BG_DARK}; width: 10px; margin: 0;
    }}
    QScrollBar::handle:vertical {{
        background: {COLOR_BG_LIGHT}; min-height: 20px; border-radius: 5px;
    }}
    QScrollBar:horizontal {{
        background: {COLOR_BG_DARK}; height: 10px; margin: 0;
    }}
    QScrollBar::handle:horizontal {{
        background: {COLOR_BG_LIGHT}; min-width: 20px; border-radius: 5px;
    }}
"""

# -------- Constantes de Items (de ambos scripts) --------
GOAL_SIZE = QtCore.QSize(50, 50)
ROBOT_SIZE = QtCore.QSize(50, 50) 

# -------- util (idénticas) --------
def point_to_str(p: QtCore.QPointF):
    return f"({int(p.x())}, {int(p.y())})"

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

# -------- items con hover y límites --------

class BoundedEllipseItem(QtWidgets.QGraphicsEllipseItem):
    """ Círculo movible (para obstáculos). """
    def __init__(self, cx, cy, r, color, name, info_cb):
        super().__init__(0, 0, 2*r, 2*r)
        self._r = r
        self._name = name
        self._info_cb = info_cb
        self.setPos(cx - r, cy - r)
        pen = QtGui.QPen(color); pen.setWidth(2)
        self.setPen(pen)
        self.setBrush(QtGui.QBrush(color))
        self.setFlag(QtWidgets.QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QtWidgets.QGraphicsItem.ItemSendsGeometryChanges, True)
        self.setAcceptHoverEvents(True)

    def center(self):
        return self.scenePos() + QtCore.QPointF(self.rect().width()/2, self.rect().height()/2)

    def hoverMoveEvent(self, e):
        c = self.center()
        if self._info_cb:
            self._info_cb(f"{self._name} en {point_to_str(c)}")

    def hoverEnterEvent(self, e): self.setOpacity(0.9)
    def hoverLeaveEvent(self, e): self.setOpacity(1.0)

    def itemChange(self, change, value):
        if change == QtWidgets.QGraphicsItem.ItemPositionChange and self.scene():
            new_pos = value
            rect = self.scene().sceneRect()
            x = clamp(new_pos.x(), rect.left(), rect.right() - self.rect().width())
            y = clamp(new_pos.y(), rect.top(), rect.bottom() - self.rect().height())
            return QtCore.QPointF(x, y)
        return super().itemChange(change, value)

class BoundedPixmapItem(QtWidgets.QGraphicsPixmapItem):
    """ Item de imagen (para Robot y Meta). """
    def __init__(self, pixmap: QtGui.QPixmap, info_cb, name="Meta"):
        super().__init__(pixmap)
        self._name = name
        self._info_cb = info_cb
        self.setFlag(QtWidgets.QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QtWidgets.QGraphicsItem.ItemSendsGeometryChanges, True)
        self.setAcceptHoverEvents(True)

    def hoverMoveEvent(self, e):
        c = self.sceneBoundingRect().center()
        if self._info_cb:
            self._info_cb(f"{self._name} en {point_to_str(c)}")

    def hoverEnterEvent(self, e): self.setOpacity(0.9)
    def hoverLeaveEvent(self, e): self.setOpacity(1.0)

    def itemChange(self, change, value):
        if change == QtWidgets.QGraphicsItem.ItemPositionChange and self.scene():
            new_pos = value
            rect = self.scene().sceneRect()
            w = self.boundingRect().width()
            h = self.boundingRect().height()
            x = clamp(new_pos.x(), rect.left(), rect.right() - w)
            y = clamp(new_pos.y(), rect.top(), rect.bottom() - h)
            return QtCore.QPointF(x, y)
        return super().itemChange(change, value)

    def center(self):
        """ Devuelve el centro del item en coordenadas de la escena. """
        w = self.boundingRect().width()
        h = self.boundingRect().height()
        return self.pos() + QtCore.QPointF(w/2, h/2)

    # --- MÉTODO AÑADIDO (CORRECCIÓN 1) ---
    def rect(self):
        """ Alias para boundingRect() para satisfacer la API del planner. """
        return self.boundingRect()
    # --- FIN DE LA ADICIÓN ---

# -------- vista del mapa con drop de obstáculos --------
class MapView(QtWidgets.QGraphicsView):
    def __init__(self, info_cb):
        super().__init__()
        self.setRenderHint(QtGui.QPainter.Antialiasing, True)
        self.setBackgroundBrush(QtGui.QBrush(QtGui.QColor(COLOR_BG_DARK)))
        self.setAcceptDrops(True)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.scene = QtWidgets.QGraphicsScene(0, 0, 680, 520)
        self.setScene(self.scene)
        self.info_cb = info_cb
        self.obstacle_count = 0
        self.obstacle_items = []

        robot_pm = self._load_robot_pixmap()
        self.robot = BoundedPixmapItem(robot_pm, self.info_cb, name="Robot")
        self.robot.setPos(80, 80)
        self.scene.addItem(self.robot)

        flag_pm = self._load_flag_pixmap()
        self.flag = BoundedPixmapItem(flag_pm, self.info_cb, name="Meta")
        self.flag.setPos(self.scene.sceneRect().right() - flag_pm.width() - 16, 16)
        self.scene.addItem(self.flag)

    def _load_robot_pixmap(self) -> QtGui.QPixmap:
        base = Path(__file__).resolve().parent
        img_dir = base / "images"
        robot_path = img_dir / "turtlebot.png"
        if robot_path.exists():
            pm = QtGui.QPixmap(str(robot_path))
            if not pm.isNull():
                return pm.scaled(ROBOT_SIZE, QtCore.Qt.KeepAspectRatio,
                                 QtCore.Qt.SmoothTransformation)
        pm = QtGui.QPixmap(ROBOT_SIZE); pm.fill(QtCore.Qt.transparent)
        p = QtGui.QPainter(pm); p.setRenderHint(QtGui.QPainter.Antialiasing, True)
        robot_color = QtGui.QColor(QtCore.Qt.green).lighter(110)
        p.setPen(QtGui.QPen(robot_color, 2))
        p.setBrush(QtGui.QBrush(robot_color))
        r = ROBOT_SIZE.width() // 2 - 2
        p.drawEllipse(pm.rect().center(), r, r)
        p.end()
        return pm

    def _load_flag_pixmap(self) -> QtGui.QPixmap:
        base = Path(__file__).resolve().parent
        img_dir = base / "images"
        candidates = [img_dir / "flag.png", img_dir / "goal.png"]
        for c in candidates:
            if c.exists():
                pm = QtGui.QPixmap(str(c))
                if not pm.isNull():
                    return pm.scaled(GOAL_SIZE, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        pm = QtGui.QPixmap(GOAL_SIZE); pm.fill(QtCore.Qt.transparent)
        p = QtGui.QPainter(pm); p.setRenderHint(QtGui.QPainter.Antialiasing, True)
        pen = QtGui.QPen(QtCore.Qt.white, 2); p.setPen(pen)
        p.drawLine(8, 6, 8, GOAL_SIZE.height()-6)
        brush = QtGui.QBrush(QtCore.Qt.red); p.setBrush(brush)
        p.drawPolygon(QtGui.QPolygon([
            QtCore.QPoint(10, 6),
            QtCore.QPoint(GOAL_SIZE.width()-10, GOAL_SIZE.height()//3),
            QtCore.QPoint(10, GOAL_SIZE.height()//3 + 6),
        ]))
        p.end()
        return pm

    def _add_obstacle_at(self, scene_pos: QtCore.QPointF, radius=16):
        r = self.scene.sceneRect()
        cx = clamp(scene_pos.x(), r.left()+radius, r.right()-radius)
        cy = clamp(scene_pos.y(), r.top()+radius, r.bottom()-radius)
        self.obstacle_count += 1
        name = f"Obstáculo #{self.obstacle_count}"
        obs_color = QtGui.QColor(QtCore.Qt.lightGray)
        item = BoundedEllipseItem(cx, cy, radius, obs_color, name, self.info_cb)
        self.scene.addItem(item)
        self.obstacle_items.append(item)
        self.info_cb(f"Añadido: {name} en {point_to_str(item.center())}")

    def dragEnterEvent(self, e):
        if e.mimeData().hasFormat("application/x-obstacle"):
            e.acceptProposedAction()
        else:
            e.ignore()

    def dragMoveEvent(self, e):
        if not e.mimeData().hasFormat("application/x-obstacle"):
            e.ignore(); return
        sp = self.mapToScene(e.pos())
        r = self.scene.sceneRect()
        radius = 16
        cx = clamp(sp.x(), r.left()+radius, r.right()-radius)
        cy = clamp(sp.y(), r.top()+radius, r.bottom()-radius)
        self.info_cb(f"Obstáculo #{self.obstacle_count + 1} ({int(cx)}, {int(cy)})")
        e.acceptProposedAction()

    def dropEvent(self, e):
        if not e.mimeData().hasFormat("application/x-obstacle"):
            e.ignore(); return
        sp = self.mapToScene(e.pos())
        self._add_obstacle_at(sp)
        e.acceptProposedAction()

    def resizeEvent(self, e):
        super().resizeEvent(e)
        w = max(100, self.viewport().width())
        h = max(100, self.viewport().height())
        self.scene.setSceneRect(0, 0, w, h)
        def reclamp(item):
            r = self.scene.sceneRect()
            br = item.boundingRect()
            x = clamp(item.pos().x(), r.left(), r.right() - br.width())
            y = clamp(item.pos().y(), r.top(), r.bottom() - br.height())
            item.setPos(x, y)
        reclamp(self.robot)
        reclamp(self.flag)
        for it in self.obstacle_items:
            reclamp(it)

# -------- icono fuente de obstáculos --------
class ObstacleSource(QtWidgets.QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setCursor(QtCore.Qt.OpenHandCursor)
        self.setFixedSize(36, 36)
        self.setScaledContents(True)
        base = Path(__file__).resolve().parent
        png_path = base / "images" / "obstacle.png"
        if png_path.exists():
            pm = QtGui.QPixmap(str(png_path)).scaled(self.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        else:
            pm = QtGui.QPixmap(self.size()); pm.fill(QtCore.Qt.transparent)
            p = QtGui.QPainter(pm); p.setRenderHint(QtGui.QPainter.Antialiasing, True)
            color = QtGui.QColor(QtCore.Qt.lightGray)
            p.setBrush(QtGui.QBrush(color))
            p.setPen(QtGui.QPen(color, 2))
            r = min(self.width(), self.height())//2 - 3
            p.drawEllipse(self.rect().center(), r, r); p.end()
        self.setPixmap(pm)

    def mousePressEvent(self, e):
        if e.button() == QtCore.Qt.LeftButton:
            self.setCursor(QtCore.Qt.ClosedHandCursor)

    def mouseMoveEvent(self, e):
        if e.buttons() & QtCore.Qt.LeftButton:
            drag = QtGui.QDrag(self)
            mime = QtCore.QMimeData()
            mime.setData("application/x-obstacle", b"1")
            drag.setMimeData(mime)
            ghost = self.pixmap().copy()
            if not ghost.isNull():
                painter = QtGui.QPainter(ghost)
                painter.setCompositionMode(QtGui.QPainter.CompositionMode_DestinationIn)
                alpha = QtGui.QColor(0,0,0, int(255*0.7))
                painter.fillRect(ghost.rect(), alpha); painter.end()
            drag.setPixmap(ghost); drag.setHotSpot(ghost.rect().center())
            drag.exec_(QtCore.Qt.CopyAction)

    def mouseReleaseEvent(self, e):
        self.setCursor(QtCore.Qt.OpenHandCursor)

# -------- botón play (del script 1, CORREGIDO) --------
class PlayButton(QtWidgets.QPushButton):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setToolTip("Ejecutar programa")
        self.setFlat(True)
        self.setObjectName("PlayButton") 
        self.setStyleSheet("""
            QPushButton#PlayButton {
                border: none;
                background: transparent;
                padding: 0;
            } 
            QPushButton#PlayButton:focus {
                outline: 0;
            }
        """)
        self.setCursor(QtCore.Qt.PointingHandCursor)
        base = Path(__file__).resolve().parent
        png_path = base / "images" / "play.png"
        TARGET_PX = 70 
        self._pm_normal = None
        self._pm_pressed = None
        
        if png_path.exists():
            original = QtGui.QPixmap(str(png_path))
            small = original.scaled(TARGET_PX, TARGET_PX, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
            self._pm_normal = small
            self._pm_pressed = self._darken(small, 0.75)
            self.setIcon(QtGui.QIcon(self._pm_normal))
            self.setIconSize(self._pm_normal.size())
            self.setText("")
            self.setFixedSize(self._pm_normal.size())
            self.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        else:
            self.setText("▶")
            f = self.font(); f.setPointSize(24); self.setFont(f)
            self.setStyleSheet(f"""
                QPushButton#PlayButton {{
                    color: {COLOR_TEXT}; 
                    border: none; 
                    background: transparent;
                    padding: 0;
                }}
                QPushButton#PlayButton:focus {{ outline: 0; }}
            """)
        self.pressed.connect(lambda: self._swap_icon(True))
        self.released.connect(lambda: self._swap_icon(False))

    def _darken(self, pix: QtGui.QPixmap, factor: float) -> QtGui.QPixmap:
        out = QtGui.QPixmap(pix.size()); out.fill(QtCore.Qt.transparent)
        p = QtGui.QPainter(out); p.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, True)
        p.drawPixmap(0, 0, pix)
        p.setCompositionMode(QtGui.QPainter.CompositionMode_SourceAtop)
        p.fillRect(out.rect(), QtGui.QColor(0, 0, 0, int(255*(1-factor))))
        p.end()
        return out

    def _swap_icon(self, pressed: bool):
        if self._pm_normal:
            self.setIcon(QtGui.QIcon(self._pm_pressed if pressed else self._pm_normal))

# -------- UI (del script 1, con estilos eliminados y CORREGIDO) --------
class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.resize(1020, 640)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        root = QtWidgets.QVBoxLayout(self.centralwidget)
        root.setContentsMargins(12,12,12,12); root.setSpacing(10)

        h_top = QtWidgets.QHBoxLayout(); h_top.setSpacing(10); root.addLayout(h_top)

        self.frame_add = QtWidgets.QFrame(); self.frame_add.setObjectName("frame_add")
        self.frame_add.setMinimumWidth(100)
        lay_add = QtWidgets.QHBoxLayout(self.frame_add); lay_add.setContentsMargins(16,8,16,8); lay_add.setSpacing(10)
        lb = QtWidgets.QLabel("Añadir obstáculo:"); f=lb.font(); f.setPointSize(11); f.setBold(True); lb.setFont(f)
        self.icon_src = ObstacleSource()
        lay_add.addWidget(lb,1); lay_add.addWidget(self.icon_src,0,QtCore.Qt.AlignRight|QtCore.Qt.AlignVCenter)
        h_top.addWidget(self.frame_add, 0)

        self.frame_info = QtWidgets.QFrame(); self.frame_info.setObjectName("frame_info")
        lay_info = QtWidgets.QHBoxLayout(self.frame_info); lay_info.setContentsMargins(16,8,16,8)
        self.lb_info = QtWidgets.QLabel("Información de interés."); self.lb_info.setMinimumWidth(380)
        self.lb_info.setAlignment(QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        lay_info.addWidget(self.lb_info,1)
        h_top.addWidget(self.frame_info, 1)

        self.frame_play = QtWidgets.QFrame(); self.frame_play.setObjectName("frame_play")
        lay_play = QtWidgets.QHBoxLayout(self.frame_play)
        self.bt_play = PlayButton(); lay_play.addWidget(self.bt_play,0,QtCore.Qt.AlignCenter)
        h_top.addWidget(self.frame_play, 0)

        h_mid = QtWidgets.QHBoxLayout(); h_mid.setSpacing(10)
        root.addLayout(h_mid, 1)

        map_frame = QtWidgets.QFrame()
        map_frame.setObjectName("map_frame")
        lay_map = QtWidgets.QVBoxLayout(map_frame)
        lay_map.setContentsMargins(4,4,4,4)
        self.view = MapView(self._set_info)
        lay_map.addWidget(self.view)
        h_mid.addWidget(map_frame, 3) 

        self.planner_panel = PlannerPanel(self.view, self._set_info)
        self.planner_panel.setObjectName("PlannerPanel") 
        h_mid.addWidget(self.planner_panel, 1)

        MainWindow.setCentralWidget(self.centralwidget)
        self.bt_play.clicked.connect(lambda: print("!En marcha!"))
        self.bt_play.clicked.connect(self.planner_panel.on_play)
        self._set_info("Pasa el ratón sobre robot/obstáculos. Arrastra el icono para añadir uno nuevo.")

    def _set_info(self, text):
        self.lb_info.setText(text)

# -------- main --------
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    
    app.setStyleSheet(DARK_MODE_STYLESHEET)
    
    w = QtWidgets.QMainWindow()
    ui = Ui_MainWindow(); ui.setupUi(w)
    
    w.setWindowTitle("Top Bar GUI – Dark Mode (Corregido)")
    
    w.show()
    sys.exit(app.exec_())