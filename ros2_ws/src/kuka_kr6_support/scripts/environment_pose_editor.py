#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
environment_pose_editor.py
──────────────────────────
Mini GUI (tkinter) para editar las poses del entorno KUKA KR6 R900.

• Carga / guarda  config/environment_poses.yaml
• Botón "Aplicar al Xacro" ejecuta apply_environment_poses.py
• No requiere ROS2 en ejecución; solo Python 3 + tkinter (preinstalado).

Uso:
    python3 scripts/environment_pose_editor.py
"""

import os
import sys
import subprocess
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import yaml
except ImportError:
    # Si PyYAML no está instalado como módulo del sistema
    sys.exit("ERROR: PyYAML requerido.  pip3 install pyyaml")


# ── Rutas ──────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT   = os.path.dirname(SCRIPT_DIR)
YAML_PATH  = os.path.join(PKG_ROOT, "config", "environment_poses.yaml")
APPLY_SCRIPT = os.path.join(SCRIPT_DIR, "apply_environment_poses.py")

PIECES = ["kuka", "base", "mesa", "gripper", "cubo"]
AXES   = ["x", "y", "z", "roll", "pitch", "yaw"]

# ── Colores tema oscuro ───────────────────────────────────────────
BG           = "#1e1e2e"
BG_CARD      = "#2a2a3c"
BG_INPUT     = "#363650"
FG           = "#cdd6f4"
FG_DIM       = "#8888aa"
ACCENT       = "#89b4fa"
ACCENT_HOVER = "#74c7ec"
GREEN        = "#a6e3a1"
RED          = "#f38ba8"
ORANGE       = "#fab387"
BORDER       = "#45475a"

PIECE_COLORS = {
    "kuka":    "#cba6f7",
    "base":    "#f5c2e7",
    "mesa":    "#89dceb",
    "gripper": "#a6e3a1",
    "cubo":    "#fab387",
}

FONT_FAMILY = "Segoe UI"  # Falls back gracefully on Linux


class PoseEditorApp:
    """Tkinter application for editing environment poses."""

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("KUKA KR6 – Environment Pose Editor")
        self.root.configure(bg=BG)
        self.root.resizable(True, True)
        self.root.minsize(700, 680)

        # Almacena StringVar para cada celda
        self.vars: dict[str, dict[str, tk.StringVar]] = {}

        self._build_ui()
        self._load_yaml()

    # ── UI ─────────────────────────────────────────────────────────

    def _build_ui(self):
        # ── Header ─────────────────────────────────────────────────
        header = tk.Frame(self.root, bg=BG)
        header.pack(fill="x", padx=20, pady=(18, 4))

        tk.Label(
            header, text="🔧  Environment Pose Editor",
            font=(FONT_FAMILY, 16, "bold"), fg=ACCENT, bg=BG,
        ).pack(side="left")

        tk.Label(
            header,
            text="kuka_kr6_support",
            font=(FONT_FAMILY, 10), fg=FG_DIM, bg=BG,
        ).pack(side="right")

        # Separator
        sep = tk.Frame(self.root, height=1, bg=BORDER)
        sep.pack(fill="x", padx=20, pady=(8, 12))

        # ── Cards container ────────────────────────────────────────
        cards_frame = tk.Frame(self.root, bg=BG)
        cards_frame.pack(fill="both", expand=True, padx=20, pady=(0, 8))

        # Hacer que las columnas se expandan
        cards_frame.columnconfigure(0, weight=1)
        cards_frame.columnconfigure(1, weight=1)

        # KUKA robot: full-width card on row 0 (spans 2 columns)
        self._build_piece_card(cards_frame, "kuka", 0, 0, colspan=2)

        # Environment pieces: 2x2 grid below
        env_positions = [(1, 0), (1, 1), (2, 0), (2, 1)]
        for idx, piece in enumerate(["base", "mesa", "gripper", "cubo"]):
            r, c = env_positions[idx]
            self._build_piece_card(cards_frame, piece, r, c)

        # ── Footer / buttons ──────────────────────────────────────
        footer = tk.Frame(self.root, bg=BG)
        footer.pack(fill="x", padx=20, pady=(4, 14))

        self._make_button(footer, "📂  Cargar YAML",   self._load_yaml,   ACCENT, side="left")
        self._make_button(footer, "💾  Guardar YAML",  self._save_yaml,   GREEN,  side="left", padx=(10, 0))
        self._make_button(footer, "⚙  Aplicar al Xacro", self._apply_xacro, ORANGE, side="left", padx=(10, 0))
        self._make_button(footer, "↺  Reset defaults", self._reset_defaults, RED,  side="right")

        # ── Status bar ─────────────────────────────────────────────
        self.status_var = tk.StringVar(value="Listo.")
        status_bar = tk.Label(
            self.root, textvariable=self.status_var,
            font=(FONT_FAMILY, 9), fg=FG_DIM, bg=BG, anchor="w",
        )
        status_bar.pack(fill="x", padx=22, pady=(0, 8))

        # ── Info label ─────────────────────────────────────────────
        info = tk.Label(
            self.root,
            text="ℹ  Para ver cambios en RViz: rebuild el paquete y relanza el launch.",
            font=(FONT_FAMILY, 9, "italic"), fg=FG_DIM, bg=BG, anchor="w",
        )
        info.pack(fill="x", padx=22, pady=(0, 10))

    def _build_piece_card(self, parent, piece: str, row: int, col: int, colspan: int = 1):
        """Construye una tarjeta visual para una pieza."""
        color = PIECE_COLORS.get(piece, ACCENT)

        card = tk.Frame(parent, bg=BG_CARD, highlightbackground=BORDER,
                        highlightthickness=1, padx=14, pady=10)
        card.grid(row=row, column=col, columnspan=colspan, padx=6, pady=6, sticky="nsew")
        parent.rowconfigure(row, weight=1)

        # Título con descripción contextual
        piece_labels = {
            "kuka":    "KUKA KR6  (posición del robot en el mundo)",
            "base":    "BASE  (pedestal)",
            "mesa":    "MESA  (mesa de trabajo)",
            "gripper": "GRIPPER  (relativo a flange)",
            "cubo":    "CUBO  (relativo a gripper)",
        }
        tk.Label(
            card, text=f"●  {piece_labels.get(piece, piece.upper())}",
            font=(FONT_FAMILY, 12, "bold"), fg=color, bg=BG_CARD, anchor="w",
        ).grid(row=0, column=0, columnspan=6, sticky="w", pady=(0, 8))

        self.vars[piece] = {}

        # xyz en una fila, rpy en la siguiente
        groups = [("x", "y", "z"), ("roll", "pitch", "yaw")]
        for g_idx, group in enumerate(groups):
            for a_idx, axis in enumerate(group):
                col_offset = a_idx * 2
                row_offset = 1 + g_idx

                lbl = tk.Label(
                    card, text=axis,
                    font=(FONT_FAMILY, 9), fg=FG_DIM, bg=BG_CARD, width=5, anchor="e",
                )
                lbl.grid(row=row_offset, column=col_offset, sticky="e", padx=(0, 2), pady=2)

                var = tk.StringVar(value="0.0")
                self.vars[piece][axis] = var

                entry = tk.Entry(
                    card, textvariable=var, width=9,
                    font=(FONT_FAMILY, 10), fg=FG, bg=BG_INPUT,
                    insertbackground=FG, relief="flat",
                    highlightbackground=BORDER, highlightthickness=1,
                    justify="center",
                )
                entry.grid(row=row_offset, column=col_offset + 1, sticky="w", padx=(0, 8), pady=2)

    def _make_button(self, parent, text, command, color, side="left", padx=0):
        """Crea un botón estilizado."""
        btn = tk.Button(
            parent, text=text, command=command,
            font=(FONT_FAMILY, 10, "bold"),
            fg=BG, bg=color, activebackground=color,
            relief="flat", cursor="hand2", padx=14, pady=5,
        )
        btn.pack(side=side, padx=padx)

        # Hover effect
        def on_enter(e, b=btn, c=color):
            b.configure(bg=self._lighten(c, 30))

        def on_leave(e, b=btn, c=color):
            b.configure(bg=c)

        btn.bind("<Enter>", on_enter)
        btn.bind("<Leave>", on_leave)

    @staticmethod
    def _lighten(hex_color: str, amount: int) -> str:
        """Aclara un color hex."""
        hex_color = hex_color.lstrip("#")
        r = min(255, int(hex_color[0:2], 16) + amount)
        g = min(255, int(hex_color[2:4], 16) + amount)
        b = min(255, int(hex_color[4:6], 16) + amount)
        return f"#{r:02x}{g:02x}{b:02x}"

    # ── Acciones ──────────────────────────────────────────────────

    def _load_yaml(self):
        """Carga valores desde environment_poses.yaml."""
        if not os.path.isfile(YAML_PATH):
            messagebox.showerror("Error", f"No se encontró:\n{YAML_PATH}")
            return

        with open(YAML_PATH, "r") as f:
            data = yaml.safe_load(f)

        if not data:
            messagebox.showwarning("Aviso", "El YAML está vacío.")
            return

        for piece in PIECES:
            piece_data = data.get(piece, {})
            for axis in AXES:
                val = piece_data.get(axis, 0.0)
                self.vars[piece][axis].set(str(val))

        self.status_var.set(f"✔  YAML cargado desde: {os.path.basename(YAML_PATH)}")

    def _save_yaml(self):
        """Guarda valores actuales a environment_poses.yaml."""
        data = {}
        for piece in PIECES:
            data[piece] = {}
            for axis in AXES:
                try:
                    data[piece][axis] = float(self.vars[piece][axis].get())
                except ValueError:
                    messagebox.showerror(
                        "Valor inválido",
                        f"{piece}.{axis} no es un número válido.",
                    )
                    return

        with open(YAML_PATH, "w") as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

        self.status_var.set(f"💾  YAML guardado en: {os.path.basename(YAML_PATH)}")

    def _apply_xacro(self):
        """Guarda el YAML y luego ejecuta apply_environment_poses.py."""
        self._save_yaml()

        if not os.path.isfile(APPLY_SCRIPT):
            messagebox.showerror("Error", f"No se encontró:\n{APPLY_SCRIPT}")
            return

        try:
            result = subprocess.run(
                [sys.executable, APPLY_SCRIPT],
                capture_output=True, text=True, cwd=PKG_ROOT,
            )
            if result.returncode == 0:
                self.status_var.set("⚙  Xacro actualizado. Relanza RViz para ver cambios.")
                messagebox.showinfo(
                    "Xacro actualizado",
                    "environment.xacro fue actualizado con los nuevos valores.\n\n"
                    "Para ver los cambios en RViz:\n"
                    "  1. colcon build --packages-select kuka_kr6_support\n"
                    "  2. source install/setup.bash\n"
                    "  3. ros2 launch kuka_kr6_support display.launch.py",
                )
            else:
                messagebox.showerror("Error al aplicar", result.stderr or result.stdout)
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _reset_defaults(self):
        """Restaura los valores por defecto."""
        defaults = {
            "kuka":    {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            "base":    {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            "mesa":    {"x": 0.8, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            "gripper": {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            "cubo":    {"x": 0.05,"y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        }
        for piece in PIECES:
            for axis in AXES:
                self.vars[piece][axis].set(str(defaults[piece][axis]))

        self.status_var.set("↺  Valores restaurados a los defaults.")


def main():
    root = tk.Tk()
    # Intentar icono si existe; si no, ignorar
    try:
        root.iconbitmap("")
    except Exception:
        pass
    app = PoseEditorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
