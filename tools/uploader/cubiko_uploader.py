#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = [
#   "PyQt6>=6.6",
#   "zeroconf>=0.130",
# ]
# ///
"""
Cubiko Uploader — drag-and-drop G-code files onto the M5DialCubikoController.

Defaults to host `cubiko.local` (mDNS). On macOS that resolves natively; on
older Windows or Linux setups, click `Scan` to discover the device's IP
via mDNS, or just type the IP in the field.

Run:
    uv run cubiko_uploader.py
or, manually:
    pip install PyQt6 zeroconf
    python3 cubiko_uploader.py
"""

from __future__ import annotations

import json
import sys
import time
from ftplib import FTP, all_errors
from pathlib import Path

from PyQt6.QtCore import Qt, QThread, pyqtSignal
from PyQt6.QtGui import QDragEnterEvent, QDropEvent
from PyQt6.QtWidgets import (
    QApplication,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

try:
    from zeroconf import ServiceBrowser, Zeroconf
    HAS_ZEROCONF = True
except ImportError:
    HAS_ZEROCONF = False

DEFAULT_HOST = "cubiko.local"
FTP_USER = "cubiko"
FTP_PASS = "cubiko"
CONFIG_PATH = Path.home() / ".cubiko_uploader.json"

DROP_NORMAL_STYLE = (
    "QLabel { background: #1e1e1e; color: #aaa; border: 2px dashed #555;"
    "         border-radius: 12px; padding: 40px; font-size: 14pt; }"
)
DROP_HOVER_STYLE = DROP_NORMAL_STYLE.replace("#555", "#0a0").replace("#aaa", "#eee")


# ---------- workers ----------

class FtpUploadThread(QThread):
    log = pyqtSignal(str)
    done = pyqtSignal(bool, str)

    def __init__(self, host: str, paths: list[str]):
        super().__init__()
        self.host = host
        self.paths = paths

    def run(self) -> None:
        try:
            ftp = FTP(self.host, timeout=15)
            ftp.login(FTP_USER, FTP_PASS)
            ftp.set_pasv(True)
            self.log.emit(f"Connected to {self.host}")
            for path in self.paths:
                name = Path(path).name
                size = Path(path).stat().st_size
                self.log.emit(f"Uploading {name} ({size} bytes)…")
                with open(path, "rb") as f:
                    ftp.storbinary(f"STOR {name}", f)
                self.log.emit(f"  ✓ {name}")
            ftp.quit()
            self.done.emit(True, f"Uploaded {len(self.paths)} file(s)")
        except all_errors as e:
            self.done.emit(False, f"FTP error: {e}")
        except OSError as e:
            self.done.emit(False, f"Connection error: {e}")


class MDnsResolver(QThread):
    """Browse _ftp._tcp.local for ~4s; emit any cubiko match."""
    resolved = pyqtSignal(str, str)  # name, ip

    def run(self) -> None:
        if not HAS_ZEROCONF:
            return
        try:
            zc = Zeroconf()
            sig = self.resolved

            class Listener:
                def add_service(self, zc, type_, name):  # noqa: N802
                    info = zc.get_service_info(type_, name, timeout=2000)
                    if not info or not info.addresses:
                        return
                    if "cubiko" not in name.lower():
                        return
                    ip = ".".join(str(b) for b in info.addresses[0])
                    sig.emit(name, ip)

                def remove_service(self, *args): pass  # noqa: N802
                def update_service(self, *args): pass  # noqa: N802

            ServiceBrowser(zc, "_ftp._tcp.local.", Listener())
            time.sleep(4)
            zc.close()
        except Exception:  # noqa: BLE001
            pass


# ---------- UI ----------

class DropZone(QLabel):
    filesDropped = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.setAcceptDrops(True)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setText("Drop G-code files here\n\nor use “Pick files…”")
        self.setStyleSheet(DROP_NORMAL_STYLE)
        self.setMinimumHeight(180)

    def dragEnterEvent(self, e: QDragEnterEvent) -> None:
        if e.mimeData().hasUrls():
            e.acceptProposedAction()
            self.setStyleSheet(DROP_HOVER_STYLE)

    def dragLeaveEvent(self, _e) -> None:
        self.setStyleSheet(DROP_NORMAL_STYLE)

    def dropEvent(self, e: QDropEvent) -> None:
        self.setStyleSheet(DROP_NORMAL_STYLE)
        paths = [u.toLocalFile() for u in e.mimeData().urls() if u.isLocalFile()]
        if paths:
            self.filesDropped.emit(paths)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Cubiko Uploader")
        self.resize(560, 520)
        cfg = self._loadConfig()

        central = QWidget()
        self.setCentralWidget(central)
        v = QVBoxLayout(central)

        # Host row
        host_row = QHBoxLayout()
        host_row.addWidget(QLabel("Device:"))
        self.hostField = QLineEdit(cfg.get("host", DEFAULT_HOST))
        host_row.addWidget(self.hostField)
        self.scanBtn = QPushButton("Scan mDNS")
        self.scanBtn.clicked.connect(self._startMDns)
        self.scanBtn.setEnabled(HAS_ZEROCONF)
        if not HAS_ZEROCONF:
            self.scanBtn.setToolTip("install `zeroconf` package to enable")
        host_row.addWidget(self.scanBtn)
        v.addLayout(host_row)

        # Drop zone
        self.drop = DropZone()
        self.drop.filesDropped.connect(self._upload)
        v.addWidget(self.drop)

        # Pick button
        pick = QPushButton("Pick files…")
        pick.clicked.connect(self._pickFiles)
        v.addWidget(pick)

        # Log
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setStyleSheet(
            "background:#111; color:#0d0; font-family:'SF Mono','Menlo',monospace;"
            "font-size:11pt;"
        )
        self.log.setMinimumHeight(140)
        v.addWidget(self.log)

        self._uploadThread: FtpUploadThread | None = None
        self._mdnsThread: MDnsResolver | None = None

    # ---- config ----

    def _loadConfig(self) -> dict:
        if CONFIG_PATH.exists():
            try:
                return json.loads(CONFIG_PATH.read_text())
            except Exception:  # noqa: BLE001
                return {}
        return {}

    def _saveConfig(self) -> None:
        try:
            CONFIG_PATH.write_text(json.dumps({"host": self.hostField.text()}))
        except Exception:  # noqa: BLE001
            pass

    # ---- helpers ----

    def _append(self, msg: str) -> None:
        self.log.append(f"[{time.strftime('%H:%M:%S')}] {msg}")

    def _pickFiles(self) -> None:
        paths, _ = QFileDialog.getOpenFileNames(self, "Pick files to upload")
        if paths:
            self._upload(paths)

    def _upload(self, paths: list[str]) -> None:
        if self._uploadThread and self._uploadThread.isRunning():
            self._append("Upload already in progress")
            return
        host = self.hostField.text().strip()
        if not host:
            self._append("Need a device hostname or IP")
            return
        self._saveConfig()
        self._uploadThread = FtpUploadThread(host, paths)
        self._uploadThread.log.connect(self._append)
        self._uploadThread.done.connect(self._onUploadDone)
        self._uploadThread.start()

    def _onUploadDone(self, ok: bool, msg: str) -> None:
        self._append(("✓ " if ok else "✗ ") + msg)

    def _startMDns(self) -> None:
        if not HAS_ZEROCONF:
            return
        self._append("Scanning mDNS for cubiko…")
        self._mdnsThread = MDnsResolver()
        self._mdnsThread.resolved.connect(self._onMDnsResolved)
        self._mdnsThread.start()

    def _onMDnsResolved(self, name: str, ip: str) -> None:
        self._append(f"Found {name} → {ip}")
        self.hostField.setText(ip)
        self._saveConfig()


def main() -> int:
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
