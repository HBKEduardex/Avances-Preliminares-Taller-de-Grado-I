#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
kuka_bridge_test_gui_node.py

GUI de prueba interna para kuka_gui_moveit_bridge.

PROPÓSITO:
  Probar la interfaz ROS 2 del bridge antes de integrar la GUI externa definitiva.
  La futura GUI externa usará exactamente los mismos tópicos.

REGLAS:
  - Trabaja exclusivamente mediante tópicos y parámetros ROS 2.
  - No importa ni llama funciones internas del bridge.
  - Todos los ángulos externos están en GRADOS.
  - No envía comandos automáticamente al iniciar.
  - No inventar valores de X, Y, Z, A, B, C al iniciar.
  - Copia el estado actual a los objetivos una sola vez al recibirlo.

ARQUITECTURA DE HILOS:
  - Hilo principal: Tkinter mainloop.
  - Hilo secundario: MultiThreadedExecutor de ROS 2.
  - Comunicación ROS→GUI: queue.Queue + root.after().
  - Publicación GUI→ROS: publisher.publish() (thread-safe en rclpy).

ADVERTENCIA:
  Esta GUI es una herramienta de prueba para RViz y MoveIt.
  No implementa conexión TCP/IP con el robot KUKA real.
  La futura GUI externa debe usar la misma interfaz de tópicos.
"""

import math
import queue
import threading
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float64MultiArray, String

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from .transform_utils import (
    JOINT_LIMITS_DEG,
    JOINT_NAMES_ORDERED,
    JOINT_LABELS,
    validate_float_array,
    validate_joint_limits,
)


# ─────────────────────────────────────────────────────────────────────────────
# Nodo ROS 2 de la GUI
# ─────────────────────────────────────────────────────────────────────────────

class KukaBridgeTestGuiNode(Node):
    """
    Nodo ROS 2 de la GUI de prueba.
    Gestiona publicadores, suscriptores y cliente de parámetros.
    Comunica datos a la GUI vía queues thread-safe.
    No accede a funciones internas del bridge.
    """

    def __init__(self):
        super().__init__('kuka_bridge_test_gui_node')

        # Carga de parámetros
        self._declare_gui_parameters()
        self._load_gui_parameters()

        # Queues para enviar datos desde callbacks ROS → hilo GUI
        self.status_queue: queue.Queue = queue.Queue()
        self.joint_state_queue: queue.Queue = queue.Queue()
        self.cartesian_state_queue: queue.Queue = queue.Queue()

        # Publicadores (GUI → Bridge)
        self._joint_cmd_pub = self.create_publisher(
            Float64MultiArray, self._joint_cmd_topic, 10)
        self._cart_cmd_pub = self.create_publisher(
            Float64MultiArray, self._cart_cmd_topic, 10)

        # Suscriptores (Bridge → GUI)
        self.create_subscription(
            String, self._status_topic,
            self._on_status, 10)
        self.create_subscription(
            Float64MultiArray, self._joint_state_topic,
            self._on_joint_state, 10)
        self.create_subscription(
            Float64MultiArray, self._cartesian_state_topic,
            self._on_cartesian_state, 10)

        # Cliente de parámetros del bridge
        self._set_param_client = self.create_client(
            SetParameters,
            f'{self._bridge_node_name}/set_parameters')

        self.get_logger().info('kuka_bridge_test_gui_node iniciado.')

    def _declare_gui_parameters(self):
        self.declare_parameter('joint_command_topic',
                               '/kuka_bridge/joint_command_deg')
        self.declare_parameter('cartesian_command_topic',
                               '/kuka_bridge/cartesian_command_deg')
        self.declare_parameter('status_topic', '/kuka_bridge/status')
        self.declare_parameter('joint_state_topic',
                               '/kuka_bridge/joint_state_deg')
        self.declare_parameter('cartesian_state_topic',
                               '/kuka_bridge/cartesian_state_deg')
        self.declare_parameter('bridge_node_name',
                               '/kuka_moveit_bridge_node')
        self.declare_parameter('linear_step_m', 0.01)
        self.declare_parameter('cartesian_angular_step_deg', 1.0)
        self.declare_parameter('joint_step_deg', 1.0)
        self.declare_parameter('auto_copy_first_joint_state', True)
        self.declare_parameter('auto_copy_first_cartesian_state', True)
        self.declare_parameter('home_joint_deg',
                               [0.0, -90.0, 90.0, 0.0, 0.0, 0.0])
        self.declare_parameter('ready_joint_deg',
                               [0.0, -28.6, 45.8, 0.0, 28.6, 0.0])
        self.declare_parameter('joint_min_deg',
                               [-170.0, -190.0, -120.0, -185.0, -120.0, -350.0])
        self.declare_parameter('joint_max_deg',
                               [170.0, 45.0, 156.0, 185.0, 120.0, 350.0])

    def _load_gui_parameters(self):
        gp = self.get_parameter
        self._joint_cmd_topic = gp('joint_command_topic').value
        self._cart_cmd_topic = gp('cartesian_command_topic').value
        self._status_topic = gp('status_topic').value
        self._joint_state_topic = gp('joint_state_topic').value
        self._cartesian_state_topic = gp('cartesian_state_topic').value
        self._bridge_node_name = gp('bridge_node_name').value
        self.linear_step_m = gp('linear_step_m').value
        self.cart_angular_step_deg = gp('cartesian_angular_step_deg').value
        self.joint_step_deg = gp('joint_step_deg').value
        self.auto_copy_joints = gp('auto_copy_first_joint_state').value
        self.auto_copy_cartesian = gp('auto_copy_first_cartesian_state').value
        self.home_joints = list(gp('home_joint_deg').value)
        self.ready_joints = list(gp('ready_joint_deg').value)
        self.joint_min = list(gp('joint_min_deg').value)
        self.joint_max = list(gp('joint_max_deg').value)

    # ── Callbacks de suscriptores ──────────────────────────────────────

    def _on_status(self, msg: String):
        self.status_queue.put(msg.data)

    def _on_joint_state(self, msg: Float64MultiArray):
        self.joint_state_queue.put(list(msg.data))

    def _on_cartesian_state(self, msg: Float64MultiArray):
        self.cartesian_state_queue.put(list(msg.data))

    # ── Publicación de comandos ────────────────────────────────────────

    def publish_joint_command(self, values_deg: list):
        """Publica comando articular en GRADOS. Thread-safe."""
        msg = Float64MultiArray()
        msg.data = [float(v) for v in values_deg]
        self._joint_cmd_pub.publish(msg)

    def publish_cartesian_command(self, values: list):
        """Publica comando cartesiano [X,Y,Z en m, A,B,C en grados]. Thread-safe."""
        msg = Float64MultiArray()
        msg.data = [float(v) for v in values]
        self._cart_cmd_pub.publish(msg)

    # ── Cliente de parámetros ─────────────────────────────────────────

    def set_bridge_parameter_async(self, name: str, value: bool):
        """Envía SetParameters al bridge de forma asíncrona."""
        if not self._set_param_client.service_is_ready():
            return None
        req = SetParameters.Request()
        param = Parameter()
        param.name = name
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_BOOL,
            bool_value=value)
        req.parameters = [param]
        return self._set_param_client.call_async(req)


# ─────────────────────────────────────────────────────────────────────────────
# GUI Tkinter
# ─────────────────────────────────────────────────────────────────────────────

class KukaTestGui:
    """
    Interfaz gráfica de prueba para el bridge.
    Se ejecuta en el hilo principal (Tkinter mainloop).
    Recibe datos de ROS via queues. Publica mediante el nodo ROS.
    """

    # Colores por estado del bridge
    STATUS_COLORS = {
        'ready':       '#27ae60',   # verde
        'succeeded':   '#27ae60',
        'already':     '#27ae60',
        'busy':        '#e67e22',   # naranja
        'planning':    '#e67e22',
        'executing':   '#e67e22',
        'computing':   '#e67e22',
        'waiting':     '#95a5a6',   # gris
        'error':       '#e74c3c',   # rojo
        'default':     '#2c3e50',   # oscuro
    }

    # Palabras clave que desactivan los botones ENVIAR
    BUSY_KEYWORDS = {
        'BUSY', 'PLANNING', 'EXECUTING', 'COMPUTING_IK',
        'WAITING_FOR_ROBOT_STATE',
    }

    # Palabras clave que reactivan los botones ENVIAR
    FREE_KEYWORDS = {
        'READY', 'SUCCEEDED', 'ALREADY_AT_TARGET',
        'IK_FAILED', 'GOAL_REJECTED', 'MOVEIT_ERROR',
    }

    def __init__(self, root: tk.Tk, node: KukaBridgeTestGuiNode):
        self.root = root
        self.node = node

        # Estado de auto-copia (solo una vez)
        self._auto_copied_joints = False
        self._auto_copied_cartesian = False

        # Último estado recibido
        self._last_joint_deg: list = None
        self._last_cartesian: list = None

        # Variables de control
        self._buttons_enabled = True
        self._plan_only_var = tk.BooleanVar(value=False)
        self._auto_send_var = tk.BooleanVar(value=False)

        # Construcción de la GUI
        self._setup_window()
        self._build_ui()

        # Iniciar polling de queues ROS
        self.root.after(100, self._poll_ros)

    def _setup_window(self):
        self.root.title('KUKA KR6 — GUI de prueba MoveIt/RViz')
        self.root.minsize(900, 700)
        self.root.configure(bg='#1a252f')
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        try:
            self.root.tk.call('tk', 'scaling', 1.1)
        except Exception:
            pass

    def _build_ui(self):
        # ── Barra de estado superior ─────────────────────────────────
        self._build_status_bar()

        # ── Frame central con dos columnas ───────────────────────────
        main_frame = tk.Frame(self.root, bg='#1a252f')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)

        # Columna izquierda: Estado actual
        left = tk.Frame(main_frame, bg='#1a252f')
        left.grid(row=0, column=0, sticky='nsew', padx=(0, 3))
        self._build_joint_state_panel(left)
        self._build_cartesian_state_panel(left)

        # Columna derecha: Objetivos
        right = tk.Frame(main_frame, bg='#1a252f')
        right.grid(row=0, column=1, sticky='nsew', padx=(3, 0))
        self._build_joint_target_panel(right)
        self._build_cartesian_target_panel(right)

        # ── Panel plan_only ──────────────────────────────────────────
        self._build_plan_only_panel()

        # ── Log de eventos ───────────────────────────────────────────
        self._build_log_panel()

    # ─── Barra de estado ──────────────────────────────────────────────

    def _build_status_bar(self):
        bar = tk.Frame(self.root, bg='#2c3e50', height=50)
        bar.pack(fill=tk.X, padx=6, pady=(6, 2))
        bar.pack_propagate(False)

        tk.Label(bar, text='KUKA KR6', font=('Courier', 13, 'bold'),
                 fg='#ecf0f1', bg='#2c3e50').pack(side=tk.LEFT, padx=10)

        self._status_indicator = tk.Label(
            bar, text='⬤ ESPERANDO', font=('Courier', 11, 'bold'),
            fg='#95a5a6', bg='#2c3e50')
        self._status_indicator.pack(side=tk.LEFT, padx=10)

        tk.Label(bar, text='Último:', font=('Courier', 9),
                 fg='#95a5a6', bg='#2c3e50').pack(side=tk.LEFT, padx=(20, 2))
        self._last_status_label = tk.Label(
            bar, text='—', font=('Courier', 9),
            fg='#bdc3c7', bg='#2c3e50', wraplength=400, justify=tk.LEFT)
        self._last_status_label.pack(side=tk.LEFT, padx=2)

    # ─── Panel: Estado articular actual ──────────────────────────────

    def _build_joint_state_panel(self, parent):
        f = self._make_frame(parent, 'ESTADO ARTICULAR ACTUAL (deg)')
        self._joint_current_vars = []
        for i in range(6):
            row = tk.Frame(f, bg='#2c3e50')
            row.pack(fill=tk.X, padx=6, pady=1)
            tk.Label(row, text=f'{JOINT_LABELS[i]}:',
                     font=('Courier', 10), fg='#95a5a6', bg='#2c3e50',
                     width=4).pack(side=tk.LEFT)
            var = tk.StringVar(value='--')
            self._joint_current_vars.append(var)
            tk.Entry(row, textvariable=var, font=('Courier', 10),
                     state='readonly', readonlybackground='#34495e',
                     fg='#1abc9c', width=12).pack(side=tk.LEFT, padx=4)

    # ─── Panel: Estado cartesiano actual ─────────────────────────────

    def _build_cartesian_state_panel(self, parent):
        f = self._make_frame(parent, 'ESTADO CARTESIANO ACTUAL (m / deg)')
        labels = ['X', 'Y', 'Z', 'A', 'B', 'C']
        units  = ['m', 'm', 'm', '°', '°', '°']
        self._cart_current_vars = []
        for i in range(6):
            row = tk.Frame(f, bg='#2c3e50')
            row.pack(fill=tk.X, padx=6, pady=1)
            tk.Label(row, text=f'{labels[i]} ({units[i]}):',
                     font=('Courier', 10), fg='#95a5a6', bg='#2c3e50',
                     width=7).pack(side=tk.LEFT)
            var = tk.StringVar(value='--')
            self._cart_current_vars.append(var)
            tk.Entry(row, textvariable=var, font=('Courier', 10),
                     state='readonly', readonlybackground='#34495e',
                     fg='#3498db', width=12).pack(side=tk.LEFT, padx=4)

    # ─── Panel: Objetivo articular ────────────────────────────────────

    def _build_joint_target_panel(self, parent):
        f = self._make_frame(parent, 'OBJETIVO ARTICULAR (deg)')

        self._joint_target_vars = []
        for i in range(6):
            row = tk.Frame(f, bg='#2c3e50')
            row.pack(fill=tk.X, padx=6, pady=1)
            tk.Label(row, text=f'{JOINT_LABELS[i]}:',
                     font=('Courier', 10), fg='#bdc3c7', bg='#2c3e50',
                     width=4).pack(side=tk.LEFT)
            var = tk.StringVar(value='')
            self._joint_target_vars.append(var)
            entry = tk.Entry(row, textvariable=var, font=('Courier', 10),
                     bg='#34495e', fg='#f1c40f', width=10)
            entry.pack(side=tk.LEFT, padx=4)
            entry.bind('<Return>', lambda e: self._send_joint_command())
            # Botones +/-
            js = self.node.joint_step_deg
            tk.Button(row, text='−', font=('Courier', 9), width=2,
                      bg='#7f8c8d', fg='white', relief='flat',
                      command=lambda idx=i: self._joint_step(idx, -1)
                      ).pack(side=tk.LEFT)
            tk.Button(row, text='+', font=('Courier', 9), width=2,
                      bg='#7f8c8d', fg='white', relief='flat',
                      command=lambda idx=i: self._joint_step(idx, +1)
                      ).pack(side=tk.LEFT, padx=(1, 0))
            lo, hi = self.node.joint_min[i], self.node.joint_max[i]
            tk.Label(row, text=f'[{lo:.0f},{hi:.0f}]',
                     font=('Courier', 8), fg='#7f8c8d', bg='#2c3e50'
                     ).pack(side=tk.LEFT, padx=4)

        btn_frame = tk.Frame(f, bg='#2c3e50')
        btn_frame.pack(fill=tk.X, padx=6, pady=4)

        self._btn_send_joints = self._make_button(
            btn_frame, 'ENVIAR A1-A6', self._send_joint_command,
            '#8e44ad')
        self._btn_send_joints.pack(side=tk.LEFT, padx=2)

        self._make_button(btn_frame, 'COPIAR JOINTS',
                          self._copy_joint_state, '#2980b9'
                          ).pack(side=tk.LEFT, padx=2)
        self._make_button(btn_frame, 'HOME',
                          self._load_home, '#16a085'
                          ).pack(side=tk.LEFT, padx=2)
        self._make_button(btn_frame, 'READY',
                          self._load_ready, '#16a085'
                          ).pack(side=tk.LEFT, padx=2)

        # Paso articular
        step_row = tk.Frame(f, bg='#2c3e50')
        step_row.pack(fill=tk.X, padx=6, pady=(0, 4))
        tk.Label(step_row, text='Paso (°):',
                 font=('Courier', 9), fg='#95a5a6', bg='#2c3e50'
                 ).pack(side=tk.LEFT)
        self._joint_step_var = tk.StringVar(
            value=str(self.node.joint_step_deg))
        tk.Entry(step_row, textvariable=self._joint_step_var,
                 font=('Courier', 9), bg='#34495e', fg='#bdc3c7',
                 width=6).pack(side=tk.LEFT, padx=4)

    # ─── Panel: Objetivo cartesiano ───────────────────────────────────

    def _build_cartesian_target_panel(self, parent):
        f = self._make_frame(parent, 'OBJETIVO CARTESIANO (m / deg)')

        labels = ['X', 'Y', 'Z', 'A', 'B', 'C']
        units  = ['m', 'm', 'm', '°', '°', '°']
        self._cart_target_vars = []
        for i in range(6):
            row = tk.Frame(f, bg='#2c3e50')
            row.pack(fill=tk.X, padx=6, pady=1)
            tk.Label(row, text=f'{labels[i]} ({units[i]}):',
                     font=('Courier', 10), fg='#bdc3c7', bg='#2c3e50',
                     width=7).pack(side=tk.LEFT)
            var = tk.StringVar(value='')
            self._cart_target_vars.append(var)
            entry = tk.Entry(row, textvariable=var, font=('Courier', 10),
                     bg='#34495e', fg='#f39c12', width=10)
            entry.pack(side=tk.LEFT, padx=4)
            entry.bind('<Return>', lambda e: self._send_cartesian_command())

        # Botones incrementales
        inc_frame = tk.Frame(f, bg='#2c3e50')
        inc_frame.pack(fill=tk.X, padx=6, pady=2)
        for col, (label, idx, step_attr) in enumerate([
            ('−X', 0, 'linear_step_m'),
            ('+X', 0, 'linear_step_m'),
            ('−Y', 1, 'linear_step_m'),
            ('+Y', 1, 'linear_step_m'),
            ('−Z', 2, 'linear_step_m'),
            ('+Z', 2, 'linear_step_m'),
        ]):
            sign = -1 if label.startswith('−') else +1
            tk.Button(
                inc_frame, text=label, font=('Courier', 8), width=3,
                bg='#1a6a9a', fg='white', relief='flat',
                command=lambda i=idx, s=sign, a=step_attr:
                    self._cart_step(i, s, a)
            ).grid(row=0, column=col, padx=1)

        inc_frame2 = tk.Frame(f, bg='#2c3e50')
        inc_frame2.pack(fill=tk.X, padx=6, pady=2)
        for col, (label, idx) in enumerate([
            ('−A', 3), ('+A', 3),
            ('−B', 4), ('+B', 4),
            ('−C', 5), ('+C', 5),
        ]):
            sign = -1 if label.startswith('−') else +1
            tk.Button(
                inc_frame2, text=label, font=('Courier', 8), width=3,
                bg='#6c3483', fg='white', relief='flat',
                command=lambda i=idx, s=sign:
                    self._cart_step(i, s, 'cart_angular_step_deg')
            ).grid(row=0, column=col, padx=1)

        # Pasos configurables
        step_row = tk.Frame(f, bg='#2c3e50')
        step_row.pack(fill=tk.X, padx=6, pady=2)
        tk.Label(step_row, text='Paso lineal (m):',
                 font=('Courier', 9), fg='#95a5a6', bg='#2c3e50'
                 ).pack(side=tk.LEFT)
        self._lin_step_var = tk.StringVar(
            value=str(self.node.linear_step_m))
        tk.Entry(step_row, textvariable=self._lin_step_var,
                 font=('Courier', 9), bg='#34495e', fg='#bdc3c7',
                 width=7).pack(side=tk.LEFT, padx=4)
        tk.Label(step_row, text='Angular (°):',
                 font=('Courier', 9), fg='#95a5a6', bg='#2c3e50'
                 ).pack(side=tk.LEFT, padx=(8, 0))
        self._ang_step_var = tk.StringVar(
            value=str(self.node.cart_angular_step_deg))
        tk.Entry(step_row, textvariable=self._ang_step_var,
                 font=('Courier', 9), bg='#34495e', fg='#bdc3c7',
                 width=7).pack(side=tk.LEFT, padx=4)

        # Botones de acción
        btn_frame = tk.Frame(f, bg='#2c3e50')
        btn_frame.pack(fill=tk.X, padx=6, pady=4)

        self._btn_send_cart = self._make_button(
            btn_frame, 'ENVIAR XYZABC', self._send_cartesian_command,
            '#c0392b')
        self._btn_send_cart.pack(side=tk.LEFT, padx=2)

        self._make_button(btn_frame, 'COPIAR POSE',
                          self._copy_cartesian_state, '#2980b9'
                          ).pack(side=tk.LEFT, padx=2)

    # ─── Panel: Plan Only ─────────────────────────────────────────────

    def _build_plan_only_panel(self):
        f = tk.Frame(self.root, bg='#2c3e50', pady=4)
        f.pack(fill=tk.X, padx=6, pady=2)

        tk.Checkbutton(
            f, text='SOLO PLANIFICAR',
            variable=self._plan_only_var,
            font=('Courier', 10), fg='#ecf0f1', bg='#2c3e50',
            selectcolor='#34495e', activebackground='#2c3e50'
        ).pack(side=tk.LEFT, padx=4)

        self._make_button(f, 'APLICAR PLAN_ONLY',
                          self._apply_plan_only, '#7f8c8d'
                          ).pack(side=tk.LEFT, padx=2)

        self._plan_only_status = tk.Label(
            f, text='', font=('Courier', 9),
            fg='#bdc3c7', bg='#2c3e50')
        self._plan_only_status.pack(side=tk.LEFT, padx=4)

        # Separador visual
        tk.Label(f, text='|', font=('Courier', 10), fg='#7f8c8d', bg='#2c3e50').pack(side=tk.LEFT, padx=6)

        # Auto send toggle
        tk.Checkbutton(
            f, text='ENVÍO AUTOMÁTICO EN TIEMPO REAL',
            variable=self._auto_send_var,
            font=('Courier', 10, 'bold'), fg='#f1c40f', bg='#2c3e50',
            selectcolor='#34495e', activebackground='#2c3e50'
        ).pack(side=tk.LEFT, padx=4)

    # ─── Panel: Log de eventos ────────────────────────────────────────

    def _build_log_panel(self):
        f = tk.LabelFrame(
            self.root, text=' REGISTRO DE EVENTOS ',
            font=('Courier', 9, 'bold'),
            fg='#95a5a6', bg='#1a252f',
            labelanchor='n', bd=1, relief='solid')
        f.pack(fill=tk.BOTH, expand=True, padx=6, pady=(2, 6))

        self._log_text = scrolledtext.ScrolledText(
            f, height=7, font=('Courier', 9),
            bg='#0d1117', fg='#58a6ff',
            insertbackground='white', state='disabled',
            wrap=tk.WORD)
        self._log_text.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        # Tags de colores
        self._log_text.tag_config('ok',    foreground='#3fb950')
        self._log_text.tag_config('warn',  foreground='#d29922')
        self._log_text.tag_config('error', foreground='#f85149')
        self._log_text.tag_config('info',  foreground='#58a6ff')
        self._log_text.tag_config('cmd',   foreground='#d2a8ff')

    # ─────────────────────────────────────────────────────────────────
    # Polling de queues ROS
    # ─────────────────────────────────────────────────────────────────

    def _poll_ros(self):
        """Procesa los datos recibidos de ROS y actualiza widgets."""
        # Estado del bridge
        while not self.node.status_queue.empty():
            try:
                status = self.node.status_queue.get_nowait()
                self._update_status(status)
            except queue.Empty:
                break

        # Estado articular
        while not self.node.joint_state_queue.empty():
            try:
                data = self.node.joint_state_queue.get_nowait()
                self._update_joint_state(data)
            except queue.Empty:
                break

        # Estado cartesiano
        while not self.node.cartesian_state_queue.empty():
            try:
                data = self.node.cartesian_state_queue.get_nowait()
                self._update_cartesian_state(data)
            except queue.Empty:
                break

        # Reprogramar
        self.root.after(100, self._poll_ros)

    # ─────────────────────────────────────────────────────────────────
    # Actualización de widgets
    # ─────────────────────────────────────────────────────────────────

    def _update_status(self, text: str):
        """Actualiza el indicador de estado y el log."""
        su = text.upper()

        if 'READY' in su and 'WAITING' not in su:
            color = self.STATUS_COLORS['ready']
            indicator = '⬤ LISTO'
        elif 'SUCCEEDED' in su or 'ALREADY_AT_TARGET' in su:
            color = self.STATUS_COLORS['succeeded']
            indicator = '⬤ OK'
        elif 'BUSY' in su or 'PLANNING' in su or 'EXECUTING' in su \
                or 'COMPUTING_IK' in su:
            color = self.STATUS_COLORS['busy']
            indicator = '⬤ OCUPADO'
        elif 'WAITING' in su:
            color = self.STATUS_COLORS['waiting']
            indicator = '⬤ ESPERANDO'
        elif any(k in su for k in ('ERROR', 'FAILED', 'REJECTED',
                                   'STALE', 'UNAVAILABLE')):
            color = self.STATUS_COLORS['error']
            indicator = '⬤ ERROR'
        else:
            color = self.STATUS_COLORS['default']
            indicator = '⬤ —'

        self._status_indicator.config(text=indicator, fg=color)
        self._last_status_label.config(
            text=text[:80] + ('...' if len(text) > 80 else ''))

        # Tag para el log
        if 'ERROR' in su or 'FAILED' in su or 'REJECTED' in su:
            tag = 'error'
        elif 'SUCCEEDED' in su or 'ALREADY_AT_TARGET' in su or \
                'IK_SUCCEEDED' in su:
            tag = 'ok'
        elif 'BUSY' in su or 'PLANNING' in su or 'EXECUTING' in su \
                or 'COMPUTING' in su:
            tag = 'warn'
        else:
            tag = 'info'

        self._log(text, tag)

        # Control de botones
        for kw in self.BUSY_KEYWORDS:
            if kw in su:
                self._set_send_buttons(False)
                return
        for kw in self.FREE_KEYWORDS:
            if kw in su:
                self._set_send_buttons(True)
                # Sincronización automática de campos
                if any(k in su for k in ('MOVEIT_ERROR', 'IK_FAILED', 'GOAL_REJECTED', 'SUCCEEDED', 'ALREADY_AT_TARGET')):
                    self._copy_joint_state()
                    self._copy_cartesian_state()
                    if any(err in su for err in ('MOVEIT_ERROR', 'IK_FAILED', 'GOAL_REJECTED')):
                        self._log('Pose revertida al estado actual debido a error.', 'info')
                return

    def _update_joint_state(self, data: list):
        """Actualiza los 6 campos de estado articular."""
        if len(data) != 6:
            return
        for i in range(6):
            self._joint_current_vars[i].set(f'{data[i]:.3f}')
        self._last_joint_deg = data

        # Auto-copia una sola vez
        if not self._auto_copied_joints and self.node.auto_copy_joints:
            self._copy_joint_state()
            self._auto_copied_joints = True

    def _update_cartesian_state(self, data: list):
        """Actualiza los 6 campos de estado cartesiano."""
        if len(data) != 6:
            return
        x, y, z, a, b, c = data
        self._cart_current_vars[0].set(f'{x:.4f}')
        self._cart_current_vars[1].set(f'{y:.4f}')
        self._cart_current_vars[2].set(f'{z:.4f}')
        self._cart_current_vars[3].set(f'{a:.3f}')
        self._cart_current_vars[4].set(f'{b:.3f}')
        self._cart_current_vars[5].set(f'{c:.3f}')
        self._last_cartesian = data

        # Auto-copia una sola vez
        if not self._auto_copied_cartesian and self.node.auto_copy_cartesian:
            self._copy_cartesian_state()
            self._auto_copied_cartesian = True

    def _set_send_buttons(self, enabled: bool):
        """Habilita/deshabilita botones ENVIAR."""
        state = tk.NORMAL if enabled else tk.DISABLED
        self._btn_send_joints.config(state=state)
        self._btn_send_cart.config(state=state)
        self._buttons_enabled = enabled

    # ─────────────────────────────────────────────────────────────────
    # Acciones de botones
    # ─────────────────────────────────────────────────────────────────

    def _send_joint_command(self):
        """Lee los 6 campos objetivo articular, valida y publica."""
        values = []
        for i, var in enumerate(self._joint_target_vars):
            try:
                v = float(var.get())
            except ValueError:
                messagebox.showerror(
                    'Error',
                    f'{JOINT_LABELS[i]}: valor no numérico: "{var.get()}"')
                return
            values.append(v)

        ok, err = validate_float_array(values, 6)
        if not ok:
            messagebox.showerror('Error de validación', err)
            return

        # Límites configurados desde YAML
        for i in range(6):
            lo, hi = self.node.joint_min[i], self.node.joint_max[i]
            if values[i] < lo or values[i] > hi:
                messagebox.showerror(
                    'Límite articular',
                    f'{JOINT_LABELS[i]}={values[i]:.3f}° fuera de '
                    f'[{lo:.1f}, {hi:.1f}]°')
                return

        self.node.publish_joint_command(values)
        cmd_str = ', '.join(
            f'{JOINT_LABELS[i]}={values[i]:.3f}°' for i in range(6))
        self._log(f'CMD ARTICULAR → {cmd_str}', 'cmd')

    def _send_cartesian_command(self):
        """Lee los 6 campos objetivo cartesiano, valida y publica."""
        labels = ['X(m)', 'Y(m)', 'Z(m)', 'A(°)', 'B(°)', 'C(°)']
        values = []
        for i, var in enumerate(self._cart_target_vars):
            try:
                v = float(var.get())
            except ValueError:
                messagebox.showerror(
                    'Error',
                    f'{labels[i]}: valor no numérico: "{var.get()}"')
                return
            values.append(v)

        ok, err = validate_float_array(values, 6)
        if not ok:
            messagebox.showerror('Error de validación', err)
            return

        self.node.publish_cartesian_command(values)
        self._log(
            f'CMD CARTESIANO → '
            f'X={values[0]:.4f}m Y={values[1]:.4f}m Z={values[2]:.4f}m '
            f'A={values[3]:.3f}° B={values[4]:.3f}° C={values[5]:.3f}°',
            'cmd')

    def _copy_joint_state(self):
        """Copia el estado articular actual a los campos objetivo."""
        if self._last_joint_deg is None:
            self._log('Sin estado articular disponible todavía.', 'warn')
            return
        for i in range(6):
            self._joint_target_vars[i].set(f'{self._last_joint_deg[i]:.3f}')
        self._log('Joints actuales copiados a objetivo.', 'info')

    def _copy_cartesian_state(self):
        """Copia la pose cartesiana actual a los campos objetivo."""
        if self._last_cartesian is None:
            self._log('Sin estado cartesiano disponible todavía.', 'warn')
            return
        fmts = ['.4f', '.4f', '.4f', '.3f', '.3f', '.3f']
        for i in range(6):
            self._cart_target_vars[i].set(
                f'{self._last_cartesian[i]:{fmts[i][1:]}}')
        self._log('Pose cartesiana actual copiada a objetivo.', 'info')

    def _load_home(self):
        """Carga posición Home en los campos objetivo (sin enviar)."""
        h = self.node.home_joints
        if len(h) != 6:
            self._log('home_joint_deg mal configurado en YAML.', 'error')
            return
        for i in range(6):
            lo, hi = self.node.joint_min[i], self.node.joint_max[i]
            if h[i] < lo or h[i] > hi:
                self._log(
                    f'Home: {JOINT_LABELS[i]}={h[i]}° fuera de límites.', 'error')
                return
        for i in range(6):
            self._joint_target_vars[i].set(f'{h[i]:.3f}')
        self._log('Posición HOME cargada en objetivo (no enviada).', 'info')

    def _load_ready(self):
        """Carga posición Ready en los campos objetivo (sin enviar)."""
        r = self.node.ready_joints
        if len(r) != 6:
            self._log('ready_joint_deg mal configurado en YAML.', 'error')
            return
        for i in range(6):
            lo, hi = self.node.joint_min[i], self.node.joint_max[i]
            if r[i] < lo or r[i] > hi:
                self._log(
                    f'Ready: {JOINT_LABELS[i]}={r[i]}° fuera de límites.', 'error')
                return
        for i in range(6):
            self._joint_target_vars[i].set(f'{r[i]:.3f}')
        self._log('Posición READY cargada en objetivo (no enviada).', 'info')

    def _joint_step(self, idx: int, sign: int):
        """Incrementa/decrementa el campo objetivo articular idx."""
        try:
            step = float(self._joint_step_var.get())
        except ValueError:
            step = self.node.joint_step_deg
        try:
            current = float(self._joint_target_vars[idx].get())
        except ValueError:
            current = 0.0
        new_val = current + sign * step
        lo, hi = self.node.joint_min[idx], self.node.joint_max[idx]
        new_val = max(lo, min(hi, new_val))
        self._joint_target_vars[idx].set(f'{new_val:.3f}')
        
        if self._auto_send_var.get() and self._buttons_enabled:
            self.root.after(10, self._send_joint_command)

    def _cart_step(self, idx: int, sign: int, step_attr: str):
        """Incrementa/decrementa el campo objetivo cartesiano idx."""
        try:
            if step_attr == 'linear_step_m':
                step = float(self._lin_step_var.get())
            else:
                step = float(self._ang_step_var.get())
        except ValueError:
            step = getattr(self.node, step_attr, 0.01)
        try:
            current = float(self._cart_target_vars[idx].get())
        except ValueError:
            current = 0.0
        new_val = current + sign * step
        self._cart_target_vars[idx].set(f'{new_val:.4f}')

        if self._auto_send_var.get() and self._buttons_enabled:
            self.root.after(10, self._send_cartesian_command)

    def _apply_plan_only(self):
        """Aplica el parámetro plan_only al bridge via servicio de parámetros."""
        value = self._plan_only_var.get()
        future = self.node.set_bridge_parameter_async('plan_only', value)
        if future is None:
            self._plan_only_status.config(
                text='❌ Servicio no disponible', fg='#e74c3c')
            self._log('ERROR: servicio de parámetros del bridge no disponible.',
                      'error')
            return
        self._plan_only_status.config(text='Aplicando...', fg='#e67e22')
        self.root.after(700, lambda: self._check_plan_only_result(future, value))

    def _check_plan_only_result(self, future, value):
        if not future.done():
            self._plan_only_status.config(text='⌛ Timeout', fg='#e74c3c')
            self._log('Timeout esperando respuesta de set_parameters.', 'warn')
            return
        try:
            result = future.result()
            if result and result.results and result.results[0].successful:
                self._plan_only_status.config(
                    text=f'✓ plan_only={value}', fg='#27ae60')
                self._log(f'plan_only={value} aplicado correctamente.', 'ok')
            else:
                reason = ''
                if result and result.results:
                    reason = result.results[0].reason
                self._plan_only_status.config(
                    text=f'❌ Rechazado', fg='#e74c3c')
                self._log(f'plan_only={value} rechazado: {reason}', 'error')
        except Exception as e:
            self._plan_only_status.config(text='❌ Error', fg='#e74c3c')
            self._log(f'Error aplicando plan_only: {e}', 'error')

    # ─────────────────────────────────────────────────────────────────
    # Log
    # ─────────────────────────────────────────────────────────────────

    def _log(self, text: str, tag: str = 'info'):
        """Agrega una línea al área de log con timestamp."""
        ts = datetime.now().strftime('%H:%M:%S')
        self._log_text.config(state='normal')
        self._log_text.insert(tk.END, f'[{ts}] {text}\n', tag)
        self._log_text.see(tk.END)
        self._log_text.config(state='disabled')

    # ─────────────────────────────────────────────────────────────────
    # Helpers de construcción UI
    # ─────────────────────────────────────────────────────────────────

    def _make_frame(self, parent, title: str) -> tk.Frame:
        outer = tk.LabelFrame(
            parent, text=f' {title} ',
            font=('Courier', 9, 'bold'),
            fg='#95a5a6', bg='#2c3e50',
            labelanchor='n', bd=1, relief='solid')
        outer.pack(fill=tk.BOTH, expand=True, padx=2, pady=3)
        return outer

    def _make_button(self, parent, text, cmd, color: str) -> tk.Button:
        return tk.Button(
            parent, text=text, command=cmd,
            font=('Courier', 9, 'bold'),
            bg=color, fg='white', relief='flat',
            padx=6, pady=3, cursor='hand2',
            activebackground=color, activeforeground='white')

    # ─────────────────────────────────────────────────────────────────
    # Cierre de ventana
    # ─────────────────────────────────────────────────────────────────

    def _on_close(self):
        """
        Cierra solamente la GUI.
        No cierra MoveIt, RViz ni el bridge.
        """
        self.root.destroy()


# ─────────────────────────────────────────────────────────────────────────────
# Punto de entrada
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = KukaBridgeTestGuiNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    root = tk.Tk()
    gui = KukaTestGui(root, node)
    gui._log('GUI de prueba iniciada. No se enviaron comandos automáticos.',
             'info')
    gui._log('Esperando estado del robot desde el bridge...', 'info')

    try:
        root.mainloop()
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
