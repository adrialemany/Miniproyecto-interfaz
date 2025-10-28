from PyQt5 import QtWidgets, QtCore

class ObjectDialog(QtWidgets.QDialog):
    """
    Diálogo para editar los parámetros de un objeto (robot u obstáculo).
    Mantiene el estilo de los diálogos previos del proyecto.
    """
    def __init__(self, item, delete_callback=None, parent=None):
        super().__init__(parent)
        self.item = item
        self.delete_callback = delete_callback
        self.setWindowTitle(f"Editar {getattr(item, '_name', 'objeto')}")
        self.setModal(True)
        self.setFixedWidth(280)

        # --- Estilo coherente con otros diálogos ---
        self.setStyleSheet("""
            QDialog {
                background-color: #2b2b2b;
                color: #f0f0f0;
            }
            QLabel {
                color: #f0f0f0;
            }
            QDoubleSpinBox {
                background-color: #3c3c3c;
                border: 1px solid #555;
                border-radius: 6px;
                color: #fff;
            }
            QPushButton {
                background-color: #0078d4;
                color: white;
                border: none;
                padding: 6px 10px;
                border-radius: 6px;
            }
            QPushButton:hover {
                background-color: #0095ff;
            }
            QPushButton:pressed {
                background-color: #005a9e;
            }
        """)

        # --- Layout general ---
        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)
        self.setLayout(layout)

        # === RADIO ===
        radio_layout = QtWidgets.QHBoxLayout()
        radio_label = QtWidgets.QLabel("Radio:")
        self.radius_spin = QtWidgets.QDoubleSpinBox()
        self.radius_spin.setRange(1, 200)
        self.radius_spin.setValue(item._r)
        self.radius_spin.valueChanged.connect(self._update_radius)
        radio_layout.addWidget(radio_label)
        radio_layout.addWidget(self.radius_spin)
        layout.addLayout(radio_layout)

        # === COORDENADAS ===
        coord_layout = QtWidgets.QGridLayout()
        coord_layout.addWidget(QtWidgets.QLabel("X:"), 0, 0)
        coord_layout.addWidget(QtWidgets.QLabel("Y:"), 1, 0)

        c = item.center()
        self.x_spin = QtWidgets.QDoubleSpinBox()
        self.x_spin.setRange(0, 2000)
        self.x_spin.setValue(c.x())
        self.x_spin.valueChanged.connect(self._update_position)

        self.y_spin = QtWidgets.QDoubleSpinBox()
        self.y_spin.setRange(0, 2000)
        self.y_spin.setValue(c.y())
        self.y_spin.valueChanged.connect(self._update_position)

        coord_layout.addWidget(self.x_spin, 0, 1)
        coord_layout.addWidget(self.y_spin, 1, 1)
        layout.addLayout(coord_layout)

        # === BOTONES ===
        btn_layout = QtWidgets.QHBoxLayout()
        self.delete_btn = QtWidgets.QPushButton("🗑 Eliminar")
        self.delete_btn.setObjectName("deleteBtn")
        self.delete_btn.clicked.connect(self._delete_item)

        self.close_btn = QtWidgets.QPushButton("Cerrar")
        self.close_btn.setObjectName("closeBtn")
        self.close_btn.clicked.connect(self.accept)

        btn_layout.addWidget(self.delete_btn)
        btn_layout.addWidget(self.close_btn)
        layout.addLayout(btn_layout)

# --- Aplicar cambios ---
    def apply_changes(self):
        """Actualiza el objeto con los valores actuales del diálogo."""
        # Actualizar radio
        r = self.radius_spin.value()
        self.item._r = r
        self.item.setRect(0, 0, 2*r, 2*r)
        # Mantiene centro en el mismo sitio
        c = QtCore.QPointF(self.x_spin.value(), self.y_spin.value())
        self.item.setPos(c.x() - r, c.y() - r)

    # --- Actualizaciones en tiempo real ---
    def _update_radius(self, val):
        """Actualiza visualmente el radio del objeto."""
        self.item._r = val
        rect = QtCore.QRectF(0, 0, 2 * val, 2 * val)
        self.item.setRect(rect)
        # Mantiene el centro en la misma posición
        c = self.item.center()
        self.item.setPos(c.x() - val, c.y() - val)

    def _update_position(self):
        """Mueve el objeto según las coordenadas."""
        new_x = self.x_spin.value()
        new_y = self.y_spin.value()
        r = self.item._r
        self.item.setPos(new_x - r, new_y - r)

    # --- Eliminar item ---
    def _delete_item(self):
        """Elimina el objeto de la escena."""
        scene = self.item.scene()
        if scene:
            scene.removeItem(self.item)

        # Si hay un callback registrado, lo ejecutamos
        if callable(self.delete_callback):
            self.delete_callback(self.item)

        self.accept()
