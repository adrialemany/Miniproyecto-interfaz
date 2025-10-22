# main_topbar_gui.py (versión final: límites en robot/meta, drop de obstáculos clampado al mapa)
import os, sys
from pathlib import Path
from PyQt5 import QtCore, QtGui, QtWidgets
from otro_planner import PlannerPanel
GOAL_SIZE = QtCore.QSize(50, 50)  # Ajusta el tamaño de la bandera

# -------- util --------
def point_to_str(p: QtCore.QPointF):
    return f"({int(p.x())}, {int(p.y())})"

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

# -------- items con hover y límites --------
class BoundedEllipseItem(QtWidgets.QGraphicsEllipseItem):
    """
    Círculo movible con hover y 'clamp' a los límites de la escena.
    name -> texto para mostrar en el panel de info.
    radius -> para calcular el margen de colisión con el borde.
    """
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
        self.setFlag(QtWidgets.QGraphicsItem.ItemSendsGeometryChanges, True)  # <- necesario para clamping
        self.setAcceptHoverEvents(True)

    def center(self):
        return self.scenePos() + QtCore.QPointF(self.rect().width()/2, self.rect().height()/2)

    def hoverMoveEvent(self, e):
        c = self.center()
        if self._info_cb:
            self._info_cb(f"{self._name} en {point_to_str(c)}")

    def hoverEnterEvent(self, e):
        self.setOpacity(0.9)

    def hoverLeaveEvent(self, e):
        self.setOpacity(1.0)

    # Limitar al rectángulo visible de la escena
    def itemChange(self, change, value):
        if change == QtWidgets.QGraphicsItem.ItemPositionChange and self.scene():
            new_pos = value
            rect = self.scene().sceneRect()
            # Mantener todo el círculo dentro: la posición del item es la esquina sup-izq del bounding
            x = clamp(new_pos.x(), rect.left(), rect.right() - self.rect().width())
            y = clamp(new_pos.y(), rect.top(), rect.bottom() - self.rect().height())
            return QtCore.QPointF(x, y)
        return super().itemChange(change, value)

class GoalFlagItem(QtWidgets.QGraphicsPixmapItem):
    """
    Bandera de meta (PNG). Movible + limitada al mapa.
    """
    def __init__(self, pixmap: QtGui.QPixmap, info_cb, name="Meta"):
        super().__init__(pixmap)
        self._name = name
        self._info_cb = info_cb
        self.setFlag(QtWidgets.QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QtWidgets.QGraphicsItem.ItemSendsGeometryChanges, True)  # <- necesario para clamping
        self.setAcceptHoverEvents(True)

    def hoverMoveEvent(self, e):
        c = self.sceneBoundingRect().center()
        if self._info_cb:
            self._info_cb(f"{self._name} en {point_to_str(c)}")

    def hoverEnterEvent(self, e):
        self.setOpacity(0.9)

    def hoverLeaveEvent(self, e):
        self.setOpacity(1.0)

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

# -------- vista del mapa con drop de obstáculos --------
class MapView(QtWidgets.QGraphicsView):
    def __init__(self, info_cb):
        super().__init__()
        self.setRenderHint(QtGui.QPainter.Antialiasing, True)
        self.setBackgroundBrush(QtGui.QBrush(QtCore.Qt.white))
        self.setAcceptDrops(True)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)

        # Escena: el rectángulo de la escena ES el área de trabajo
        self.scene = QtWidgets.QGraphicsScene(0, 0, 680, 520)
        self.setScene(self.scene)

        self.info_cb = info_cb
        self.obstacle_count = 0
        self.obstacle_items = []

        # Robot por defecto
        self.robot = BoundedEllipseItem(80, 80, 14, QtCore.Qt.darkGreen, "Robot", self.info_cb)
        self.scene.addItem(self.robot)

        # Bandera de meta
        flag_pm = self._load_flag_pixmap()
        self.flag = GoalFlagItem(flag_pm, self.info_cb, name="Meta")
        # posición inicial: esquina sup-der con margen
        self.flag.setPos(self.scene.sceneRect().right() - flag_pm.width() - 16, 16)
        self.scene.addItem(self.flag)

    def _load_flag_pixmap(self) -> QtGui.QPixmap:
        base = Path(__file__).resolve().parent
        img_dir = base / "images"
        candidates = [img_dir / "flag.png", img_dir / "goal.png"]
        for c in candidates:
            if c.exists():
                pm = QtGui.QPixmap(str(c))
                if not pm.isNull():
                    # Escalado de la imagen de meta
                    return pm.scaled(GOAL_SIZE, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        # Fallback dibujando banderita ya con ese tamaño
        pm = QtGui.QPixmap(GOAL_SIZE); pm.fill(QtCore.Qt.transparent)
        p = QtGui.QPainter(pm); p.setRenderHint(QtGui.QPainter.Antialiasing, True)
        pen = QtGui.QPen(QtCore.Qt.black, 2); p.setPen(pen)
        p.drawLine(8, 6, 8, GOAL_SIZE.height()-6)
        brush = QtGui.QBrush(QtCore.Qt.red); p.setBrush(brush)
        p.drawPolygon(QtGui.QPolygon([
            QtCore.QPoint(10, 6),
            QtCore.QPoint(GOAL_SIZE.width()-10, GOAL_SIZE.height()//3),
            QtCore.QPoint(10, GOAL_SIZE.height()//3 + 6),
        ]))
        p.end()
        return pm

    # Crear obstáculo respetando límites
    def _add_obstacle_at(self, scene_pos: QtCore.QPointF, radius=16):
        r = self.scene.sceneRect()
        # clamp al centro para que el círculo no se salga
        cx = clamp(scene_pos.x(), r.left()+radius, r.right()-radius)
        cy = clamp(scene_pos.y(), r.top()+radius, r.bottom()-radius)
        self.obstacle_count += 1
        name = f"Obstáculo #{self.obstacle_count}"
        item = BoundedEllipseItem(cx, cy, radius, QtCore.Qt.darkGray, name, self.info_cb)
        self.scene.addItem(item)
        self.obstacle_items.append(item)
        self.info_cb(f"Añadido: {name} en {point_to_str(item.center())}")

    # Drag desde el icono (acepta y clampa en tiempo real)
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
        # Siempre clampa dentro antes de crear
        self._add_obstacle_at(sp)
        e.acceptProposedAction()

    def resizeEvent(self, e):
        super().resizeEvent(e)
        # nuevo rectángulo de escena: ocupa todo el viewport
        w = max(100, self.viewport().width())
        h = max(100, self.viewport().height())
        self.scene.setSceneRect(0, 0, w, h)

        # re-clamp de todos los items existentes al nuevo tamaño
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
            p.setBrush(QtGui.QBrush(QtCore.Qt.darkGray))
            p.setPen(QtGui.QPen(QtCore.Qt.darkGray, 2))
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
                alpha = QtGui.QColor(0,0,0, int(255*0.6))  # 60%
                painter.fillRect(ghost.rect(), alpha); painter.end()
            drag.setPixmap(ghost); drag.setHotSpot(ghost.rect().center())
            drag.exec_(QtCore.Qt.CopyAction)

    def mouseReleaseEvent(self, e):
        self.setCursor(QtCore.Qt.OpenHandCursor)

# -------- botón play (solo PNG, sin contorno; pequeño y se oscurece al pulsar) --------
class PlayButton(QtWidgets.QPushButton):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setToolTip("Ejecutar programa")
        self.setFlat(True)
        self.setStyleSheet("QPushButton{border:none;background:transparent;padding:0;} QPushButton:focus{outline:0;}")
        self.setCursor(QtCore.Qt.PointingHandCursor)
        base = Path(__file__).resolve().parent
        png_path = base / "images" / "play.png"
        # === AJUSTA AQUÍ EL TAMAÑO MÁXIMO DEL ICONO (en píxeles) ===
        TARGET_PX = 70  # pon 24–36 según quieras más pequeño o grande
        # ============================================================
        self._pm_normal = None
        self._pm_pressed = None
        if png_path.exists():
            original = QtGui.QPixmap(str(png_path))
            small = original.scaled(TARGET_PX, TARGET_PX, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
            self._pm_normal = small
            self._pm_pressed = self._darken(small, 0.75)  # oscurecer al pulsar
            self.setIcon(QtGui.QIcon(self._pm_normal))
            self.setIconSize(self._pm_normal.size())
            self.setText("")
            self.setFixedSize(self._pm_normal.size())
            self.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        else:
            self.setText("▶")
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

# -------- UI estilo Qt Designer --------
class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.resize(1020, 640)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        root = QtWidgets.QVBoxLayout(self.centralwidget)
        root.setContentsMargins(12,12,12,12); root.setSpacing(10)

        # --- fila superior (3 bloques) ---
        h_top = QtWidgets.QHBoxLayout(); h_top.setSpacing(10); root.addLayout(h_top)

        # Bloque 1: fuente de obstáculos
        self.frame_add = QtWidgets.QFrame(); self.frame_add.setObjectName("frame_add")
        self.frame_add.setStyleSheet("QFrame#frame_add{border:2px solid #444;border-radius:14px;background:#f7f7f7;}")
        self.frame_add.setMinimumWidth(100)
        lay_add = QtWidgets.QHBoxLayout(self.frame_add); lay_add.setContentsMargins(16,8,16,8); lay_add.setSpacing(10)
        lb = QtWidgets.QLabel("Añadir obstáculo:"); f=lb.font(); f.setPointSize(11); f.setBold(True); lb.setFont(f)
        self.icon_src = ObstacleSource()
        lay_add.addWidget(lb,1); lay_add.addWidget(self.icon_src,0,QtCore.Qt.AlignRight|QtCore.Qt.AlignVCenter)
        h_top.addWidget(self.frame_add, 0)

        # Bloque 2: info
        self.frame_info = QtWidgets.QFrame(); self.frame_info.setObjectName("frame_info")
        self.frame_info.setStyleSheet("QFrame#frame_info{border:2px solid #444;border-radius:14px;background:#f7f7f7;}")
        lay_info = QtWidgets.QHBoxLayout(self.frame_info); lay_info.setContentsMargins(16,8,16,8)
        self.lb_info = QtWidgets.QLabel("Información de interés."); self.lb_info.setMinimumWidth(380)
        self.lb_info.setAlignment(QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        lay_info.addWidget(self.lb_info,1)
        h_top.addWidget(self.frame_info, 1)

        # Bloque 3: Play
        self.frame_play = QtWidgets.QFrame(); self.frame_play.setObjectName("frame_play")
        lay_play = QtWidgets.QHBoxLayout(self.frame_play)
        self.bt_play = PlayButton(); lay_play.addWidget(self.bt_play,0,QtCore.Qt.AlignCenter)
        h_top.addWidget(self.frame_play, 0)

        # --- zona media: mapa (izda) + panel planner (dcha) ---
        h_mid = QtWidgets.QHBoxLayout(); h_mid.setSpacing(10)
        root.addLayout(h_mid, 1)

        # Mapa (izquierda)
        map_frame = QtWidgets.QFrame()
        map_frame.setStyleSheet("QFrame{border:2px solid #444;border-radius:14px;}")
        lay_map = QtWidgets.QVBoxLayout(map_frame); lay_map.setContentsMargins(8,8,8,8)
        self.view = MapView(self._set_info)
        lay_map.addWidget(self.view)
        h_mid.addWidget(map_frame, 3)  # ~2/3 del ancho

        # Panel planner (derecha)
        self.planner_panel = PlannerPanel(self.view, self._set_info)
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
    w = QtWidgets.QMainWindow()
    ui = Ui_MainWindow(); ui.setupUi(w)
    w.setWindowTitle("Top Bar GUI – límites + meta")
    w.show()
    sys.exit(app.exec_())
