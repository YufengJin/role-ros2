#!/usr/bin/env python3
"""
Docker-ROS Control Center (Config Driven)
Reads 'config.json' to dynamically generate controls.
"""

import sys
import subprocess
import time
import json
import os
from typing import Optional, Dict, List
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QTableWidget, QTableWidgetItem, QTextEdit,
    QLabel, QMessageBox, QHeaderView, QAbstractItemView
)
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QFont, QColor
import psutil

# ============================================================================
# Worker Thread
# ============================================================================

class ContainerWorker(QThread):
    log_received = pyqtSignal(str)
    status_changed = pyqtSignal(bool)
    
    def __init__(self, config: dict):
        super().__init__()
        self.cfg = config
        self.container_name = config['container']
        # Build ROS2 environment setup command
        # Try to detect ROS2 version and source appropriate setup files
        # This works for both Foxy (ros2_polymetis) and Humble (ros2_cu118) containers
        # Order: try Foxy first (for ros2_polymetis_container), then Humble (for ros2_cu118_container)
        ros2_setup = (
            "if [ -f /opt/ros/foxy/setup.bash ]; then "
            "  source /opt/ros/foxy/setup.bash; "
            "elif [ -f /opt/ros/humble/setup.bash ]; then "
            "  source /opt/ros/humble/setup.bash; "
            "else "
            "  source /opt/ros/*/setup.bash 2>/dev/null || true; "
            "fi"
        )
        workspace_setup = "[ -f /app/ros2_ws/install/setup.bash ] && source /app/ros2_ws/install/setup.bash || true"
        # Combine: source ROS2, source workspace, then run command
        self.start_cmd = f"{ros2_setup} && {workspace_setup} && {config['command']}"
        self.kill_keyword = config['kill_keyword']
        self.tag = f"[{config['name']}]"
        self.process: Optional[subprocess.Popen] = None
        self.is_running = False
        self._stop_requested = False
    
    def run(self):
        """Start the process inside docker."""
        if not self._check_container_running():
            self.log_received.emit(f"{self.tag} Error: Container '{self.container_name}' is not running.")
            self.status_changed.emit(False)
            return

        self.is_running = True
        self.status_changed.emit(True)
        self._stop_requested = False
        
        # Use stdbuf -o0 to force unbuffered output from the container
        docker_cmd = [
            "docker", "exec", "-i", self.container_name,
            "bash", "-c", self.start_cmd
        ]
        
        try:
            self.log_received.emit(f"{self.tag} Starting...")
            self.process = subprocess.Popen(
                docker_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1
            )
            
            for line in iter(self.process.stdout.readline, ''):
                if self._stop_requested: break
                if line: self.log_received.emit(f"{self.tag} {line.rstrip()}")
            
            self.process.wait()
            self.log_received.emit(f"{self.tag} Process exited.")
        
        except Exception as e:
            self.log_received.emit(f"{self.tag} Error: {str(e)}")
        finally:
            self.is_running = False
            self.status_changed.emit(False)

    def stop_process(self):
        """Correctly kill the process INSIDE the container."""
        if not self.is_running: return
        self._stop_requested = True
        
        self.log_received.emit(f"{self.tag} Sending SIGINT (Ctrl+C)...")
        
        # 1. First attempt: Graceful Shutdown (SIGINT)
        # We use pkill -f to match the command line arguments
        subprocess.run([
            "docker", "exec", self.container_name,
            "bash", "-c", f"pkill -SIGINT -f '{self.kill_keyword}'"
        ])
        
        # 2. Wait a moment for graceful shutdown
        # Check if process is still running inside container using pgrep
        for _ in range(10): # Wait up to 2 seconds (10 * 0.2)
            if not self._check_process_exists_in_container():
                self.log_received.emit(f"{self.tag} Stopped gracefully.")
                return
            time.sleep(0.2)

        # 3. Force Kill (SIGKILL) if still running
        self.log_received.emit(f"{self.tag} Process stuck. Sending SIGKILL...")
        subprocess.run([
            "docker", "exec", self.container_name,
            "bash", "-c", f"pkill -SIGKILL -f '{self.kill_keyword}'"
        ])

    def _check_container_running(self) -> bool:
        try:
            res = subprocess.run(["docker", "ps", "--format", "{{.Names}}"], capture_output=True, text=True)
            return self.container_name in res.stdout
        except: return False

    def _check_process_exists_in_container(self) -> bool:
        """Check if the specific process is still running inside docker."""
        try:
            # pgrep returns 0 if found, 1 if not
            cmd = [
                "docker", "exec", self.container_name, 
                "bash", "-c", f"pgrep -f '{self.kill_keyword}'"
            ]
            res = subprocess.run(cmd, capture_output=True)
            return res.returncode == 0
        except:
            return False

# ============================================================================
# Main GUI
# ============================================================================

class DockerROSControlCenter(QMainWindow):
    def __init__(self):
        super().__init__()
        self.workers = {} # Key: Config Name, Value: Worker
        self.configs = self.load_config()
        self.init_ui()
        
    def load_config(self) -> List[dict]:
        """Load config.json from the same directory."""
        config_path = os.path.join(os.path.dirname(__file__), 'config.json')
        if not os.path.exists(config_path):
            QMessageBox.critical(self, "Error", f"Config file not found:\n{config_path}")
            sys.exit(1)
        try:
            with open(config_path, 'r') as f:
                return json.load(f)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Invalid JSON:\n{e}")
            sys.exit(1)

    def init_ui(self):
        self.setWindowTitle("Docker-ROS Control Center (Dynamic)")
        self.setGeometry(100, 100, 1000, 800)
        # Light theme styles
        self.setStyleSheet("""
            QMainWindow { 
                background-color: #f5f5f5; 
                color: #333333; 
            }
            QPushButton {
                background-color: #e0e0e0;
                border: 1px solid #b0b0b0;
                border-radius: 4px;
                padding: 6px 12px;
                color: #333333;
            }
            QPushButton:hover {
                background-color: #d0d0d0;
            }
            QPushButton:pressed {
                background-color: #c0c0c0;
            }
        """)
        
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        # --- Top Controls ---
        top_layout = QHBoxLayout()
        self.lbl_info = QLabel(f"Loaded {len(self.configs)} configurations")
        self.lbl_info.setStyleSheet("color: #666666;")
        btn_start_all = QPushButton("Start All")
        btn_stop_all = QPushButton("Stop All")
        btn_start_all.clicked.connect(self.start_all)
        btn_stop_all.clicked.connect(self.stop_all)
        
        top_layout.addWidget(self.lbl_info)
        top_layout.addStretch()
        top_layout.addWidget(btn_start_all)
        top_layout.addWidget(btn_stop_all)
        layout.addLayout(top_layout)

        # --- Dynamic Table ---
        self.table = QTableWidget(len(self.configs), 4)
        self.table.setHorizontalHeaderLabels(["Name", "Container", "Status", "Controls"])
        self.table.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.setStyleSheet("""
            QTableWidget {
                background-color: #ffffff;
                gridline-color: #d0d0d0;
                color: #333333;
            }
            QHeaderView::section {
                background-color: #e8e8e8;
                color: #333333;
                padding: 4px;
                border: 1px solid #d0d0d0;
            }
        """)
        
        for i, cfg in enumerate(self.configs):
            # Name
            self.table.setItem(i, 0, QTableWidgetItem(cfg['name']))
            # Container
            self.table.setItem(i, 1, QTableWidgetItem(cfg['container']))
            # Status
            status_item = QTableWidgetItem("Stopped")
            status_item.setForeground(QColor("#d32f2f"))  # Darker red for light theme
            self.table.setItem(i, 2, status_item)
            
            # Buttons
            btn_widget = QWidget()
            btn_layout = QHBoxLayout(btn_widget)
            btn_layout.setContentsMargins(0, 0, 0, 0)
            
            b_start = QPushButton("Start")
            b_stop = QPushButton("Stop")
            
            # 使用 closure 绑定当前的配置
            b_start.clicked.connect(lambda checked, idx=i: self.start_task(idx))
            b_stop.clicked.connect(lambda checked, idx=i: self.stop_task(idx))
            
            btn_layout.addWidget(b_start)
            btn_layout.addWidget(b_stop)
            self.table.setCellWidget(i, 3, btn_widget)

        layout.addWidget(self.table)

        # --- Logs ---
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setStyleSheet("""
            background-color: #ffffff;
            color: #2e7d32;
            font-family: monospace;
            border: 1px solid #d0d0d0;
        """)
        layout.addWidget(self.log_view)

    def log(self, msg):
        self.log_view.append(msg)
        self.log_view.verticalScrollBar().setValue(self.log_view.verticalScrollBar().maximum())

    def update_table_status(self, index, is_running):
        item = self.table.item(index, 2)
        if is_running:
            item.setText("Running")
            item.setForeground(QColor("#388e3c"))  # Darker green for light theme
        else:
            item.setText("Stopped")
            item.setForeground(QColor("#d32f2f"))  # Darker red for light theme

    def start_task(self, index):
        cfg = self.configs[index]
        name = cfg['name']
        
        if name in self.workers and self.workers[name].is_running:
            self.log(f"[{name}] Already running.")
            return

        worker = ContainerWorker(cfg)
        worker.log_received.connect(self.log)
        worker.status_changed.connect(lambda s, idx=index: self.update_table_status(idx, s))
        
        self.workers[name] = worker
        worker.start()

    def stop_task(self, index):
        cfg = self.configs[index]
        name = cfg['name']
        if name in self.workers:
            self.workers[name].stop_process()

    def start_all(self):
        self.log("=== Starting All Tasks ===")
        # 依次启动，中间加一点延迟防止瞬间负载过高
        for i in range(len(self.configs)):
            self.start_task(i)
            QApplication.processEvents() # 刷新UI
            time.sleep(0.5)

    def stop_all(self):
        self.log("=== Stopping All Tasks ===")
        for name, worker in self.workers.items():
            worker.stop_process()
            
    def closeEvent(self, event):
        self.stop_all()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = DockerROSControlCenter()
    win.show()
    sys.exit(app.exec_())