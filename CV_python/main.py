# -*- coding: utf-8 -*-
"""
ParisBioTek - Pilote de voltampérométrie cyclique (CV) — transport SMP/mcumgr
=============================================================================

Pilote la carte nRF52840 + AD5940 par liaison série USB, EN PARTAGEANT le canal
avec le DFU mcumgr : les commandes CV passent par un groupe SMP custom (ID 64),
donc pas de conflit avec le DFU (même port, un seul propriétaire à la fois).

Flux :
  - START  (groupe 64, cmd 0, write) : règle les paramètres + lance la CV.
  - STATUS (groupe 64, cmd 1, read)  : { run, n, peak_na, r_ohm, rtia }.
  - DATA   (groupe 64, cmd 2, read)  : { off } -> { off, n, v[], i[] } par lots.

Dépendances :  pip install pyserial cbor2 matplotlib
(tkinter est fourni avec Python sous Windows.)

Le port à choisir est CELUI du DFU/mcumgr (le même que flash.ps1). Ferme toute
autre appli qui le tient (mcumgr-client, terminal série) avant de connecter.
"""

import csv
import os
import re
import queue
import struct
import base64
import threading
import time
from datetime import datetime

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    raise SystemExit("Module 'pyserial' requis :  pip install pyserial")

try:
    import cbor2
except ImportError:
    raise SystemExit("Module 'cbor2' requis :  pip install cbor2")

try:
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
except ImportError:
    raise SystemExit("Module 'matplotlib' requis :  pip install matplotlib")

# ─────────────────────────────────────────────────────────────────────────────
# Transport SMP sur série (framing mcumgr identique au firmware Zephyr)
# ─────────────────────────────────────────────────────────────────────────────
SMP_OP_READ, SMP_OP_WRITE = 0, 2
CV_GROUP = 64
CV_CMD_START, CV_CMD_STATUS, CV_CMD_DATA = 0, 1, 2

_HDR_PKT = b"\x06\x09"     # marqueur 1er fragment
_HDR_FRAG = b"\x04\x14"    # marqueur fragment suivant


def crc16_ccitt(data: bytes) -> int:
    """CRC16-ITU-T / XMODEM : poly 0x1021, init 0x0000, MSB d'abord (= firmware)."""
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


class SmpError(Exception):
    pass


class SmpSerial:
    """Client SMP minimal : requête/réponse framées base64 + CRC sur le port série."""

    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.4, write_timeout=2.0)
        self.seq = 0

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    @staticmethod
    def _frame(packet: bytes) -> bytes:
        crc = crc16_ccitt(packet)
        body = struct.pack(">H", len(packet) + 2) + packet + struct.pack(">H", crc)
        return _HDR_PKT + base64.b64encode(body) + b"\n"

    def _read_packet(self, timeout: float) -> bytes:
        deadline = time.time() + timeout
        buf = b""
        expected = None
        while time.time() < deadline:
            line = self.ser.readline()
            if not line:
                continue
            line = line.strip(b"\r\n")
            if line.startswith(_HDR_PKT):
                try:
                    dec = base64.b64decode(line[2:])
                except Exception:
                    continue
                if len(dec) < 2:
                    continue
                expected = struct.unpack(">H", dec[:2])[0]
                buf = dec[2:]
            elif line.startswith(_HDR_FRAG):
                try:
                    buf += base64.b64decode(line[2:])
                except Exception:
                    continue
            else:
                continue  # ligne non-SMP (bruit) : ignorée
            if expected is not None and len(buf) >= expected:
                data = buf[:expected]
                packet, crc_rx = data[:-2], struct.unpack(">H", data[-2:])[0]
                if crc16_ccitt(packet) != crc_rx:
                    raise SmpError("CRC SMP invalide")
                return packet
        raise SmpError("pas de réponse SMP (timeout)")

    def request(self, op, group, cmd_id, payload: dict, timeout=4.0) -> dict:
        cbor_payload = cbor2.dumps(payload)
        seq = self.seq
        self.seq = (self.seq + 1) & 0xFF
        hdr = struct.pack(">BBHHBB", op, 0, len(cbor_payload), group, seq, cmd_id)
        packet = hdr + cbor_payload
        self.ser.reset_input_buffer()
        self.ser.write(self._frame(packet))
        # Lit jusqu'à la réponse du bon groupe/commande.
        deadline = time.time() + timeout
        while time.time() < deadline:
            rsp = self._read_packet(max(0.2, deadline - time.time()))
            if len(rsp) < 8:
                continue
            r_op, _flags, r_len, r_group, _seq, r_id = struct.unpack(">BBHHBB", rsp[:8])
            body = rsp[8:8 + r_len]
            if r_group != group or r_id != cmd_id:
                continue  # pas notre réponse (autre groupe = ex. DFU) : on ignore
            obj = cbor2.loads(body) if body else {}
            if isinstance(obj, dict) and obj.get("rc", 0) not in (0, None):
                raise SmpError(f"erreur SMP rc={obj.get('rc')}")
            return obj
        raise SmpError("pas de réponse SMP attendue (timeout)")


# ── Commandes CV ──────────────────────────────────────────────────────────────
def cv_start(smp, params):
    return smp.request(SMP_OP_WRITE, CV_GROUP, CV_CMD_START, params)


def cv_status(smp):
    return smp.request(SMP_OP_READ, CV_GROUP, CV_CMD_STATUS, {})


def cv_data(smp, off):
    return smp.request(SMP_OP_READ, CV_GROUP, CV_CMD_DATA, {"off": off})


# ─────────────────────────────────────────────────────────────────────────────
# Paramètres et gamme RTIA
# ─────────────────────────────────────────────────────────────────────────────
DEFAULTS = {
    "start_mv": -1000.0, "peak_mv": 1000.0, "vzero_mv": 1300.0,
    "steps": 800, "duration_ms": 24000, "sample_delay_ms": 7.0,
}

# Méthodes d'analyse — les entiers DOIVENT correspondre à methods.h (firmware).
METHOD_CV, METHOD_CA, METHOD_DPV = 0, 1, 2
METHOD_LABELS = [
    ("Voltampérométrie cyclique (CV)",  METHOD_CV),
    ("Chronoampérométrie (i–t)",        METHOD_CA),
    ("Voltamp. impulsions diff. (DPV)", METHOD_DPV),
]
CA_DEFAULTS = {"pot_mv": 500.0, "vzero_mv": 1300.0, "duration_ms": 10000, "interval_ms": 100}
DPV_DEFAULTS = {"start_mv": -600.0, "end_mv": 600.0, "estep_mv": 5.0, "epulse_mv": 50.0,
                "tpulse_ms": 50, "tperiod_ms": 200, "vzero_mv": 1300.0}

# Libellés lisibles pour les clés de paramètres (fiche .txt).
KEY_LABELS = {
    "start_mv": ("Tension départ", "mV"), "peak_mv": ("Tension sommet", "mV"),
    "end_mv": ("Tension fin", "mV"), "vzero_mv": ("Vzero / biais", "mV"),
    "steps": ("Nombre de pas", ""), "dur_ms": ("Durée", "ms"),
    "delay_us": ("Délai échantillon", "µs"), "rtia": ("RTIA demandé", "Ω"),
    "ca_pot_mv": ("Potentiel", "mV"), "ca_dur_ms": ("Durée", "ms"),
    "ca_interval_ms": ("Intervalle", "ms"),
    "dpv_estep_mv": ("Hauteur de marche", "mV"), "dpv_epulse_mv": ("Amplitude impulsion", "mV"),
    "dpv_tpulse_ms": ("Durée impulsion", "ms"), "dpv_tperiod_ms": ("Période/marche", "ms"),
}

RTIA_VALUES = [200, 1000, 2000, 3000, 4000, 6000, 8000, 10000, 12000, 16000,
               20000, 24000, 30000, 32000, 40000, 48000, 64000, 85000, 96000,
               100000, 120000, 128000, 160000, 196000, 256000, 512000]
RTIA_AUTO_LABEL = "Auto (auto-range)"


def _rtia_label(ohm):
    return f"{ohm / 1000:g} kΩ" if ohm >= 1000 else f"{ohm} Ω"


RTIA_LABEL_TO_OHM = {RTIA_AUTO_LABEL: 0}
RTIA_LABEL_TO_OHM.update({_rtia_label(o): o for o in RTIA_VALUES})


def _rtia_label_for_ohm(ohm):
    if ohm <= 0:
        return RTIA_AUTO_LABEL
    return _rtia_label(min(RTIA_VALUES, key=lambda v: abs(v - ohm)))


def _safe_name(name, fallback):
    """Nettoie un nom pour en faire un composant de chemin valide sous Windows."""
    name = re.sub(r'[<>:"/\\|?*\x00-\x1f]+', "_", (name or "").strip())
    name = name.strip(" .")            # pas d'espace/point en fin (interdits Windows)
    return name or fallback


# ─────────────────────────────────────────────────────────────────────────────
# Application
# ─────────────────────────────────────────────────────────────────────────────
class CVApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ParisBioTek — Analyse électrochimique (SMP)")
        self.geometry("1080x700")
        self.minsize(900, 560)

        self.smp = None
        self.q = queue.Queue()
        self.busy = False               # une opération série est en cours (thread)
        self.points = []                # (index, volt_mV, current_nA)
        self.last_result = ""
        self.last_params = None         # params envoyés au firmware (dernier run)
        self.last_status = {}           # STATUS renvoyé après le dernier run
        self.last_run_dt = None         # horodatage de la mesure
        self.last_rtia_label = ""       # gamme RTIA demandée (libellé)
        self.last_meta = None           # méta d'affichage/CSV du dernier run

        self._build_ui()
        self._refresh_ports()
        self.after(60, self._poll_queue)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── UI ────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        # Créée tôt : _read_params() (via _update_duration_label) la lit avant
        # que le cadre RTIA plus bas ne soit construit.
        self.rtia_var = tk.StringVar(value=RTIA_AUTO_LABEL)
        root = ttk.Frame(self, padding=10)
        root.pack(fill="both", expand=True)
        root.columnconfigure(1, weight=1)
        root.rowconfigure(0, weight=1)
        left = ttk.Frame(root)
        left.grid(row=0, column=0, sticky="ns", padx=(0, 10))

        conn = ttk.LabelFrame(left, text="Connexion (port DFU/mcumgr)", padding=8)
        conn.pack(fill="x", pady=(0, 8))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, width=22, state="readonly")
        self.port_combo.grid(row=0, column=0, columnspan=2, sticky="ew", pady=2)
        ttk.Button(conn, text="Rafraîchir", command=self._refresh_ports)\
            .grid(row=1, column=0, sticky="ew", padx=(0, 4), pady=2)
        self.btn_detect = ttk.Button(conn, text="Détecter", command=self._autodetect)
        self.btn_detect.grid(row=1, column=1, sticky="ew", pady=2)
        self.btn_connect = ttk.Button(conn, text="Connecter", command=self._toggle_connect)
        self.btn_connect.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(4, 0))
        conn.columnconfigure(0, weight=1)
        conn.columnconfigure(1, weight=1)

        sess = ttk.LabelFrame(left, text="Session de test", padding=8)
        sess.pack(fill="x", pady=(0, 8))
        ttk.Label(sess, text="Nom de session").grid(row=0, column=0, sticky="w", pady=2)
        self.session_var = tk.StringVar()
        ttk.Entry(sess, textvariable=self.session_var, width=18)\
            .grid(row=0, column=1, sticky="ew", pady=2)
        ttk.Label(sess, text="Nom du test").grid(row=1, column=0, sticky="w", pady=2)
        self.test_var = tk.StringVar()
        ttk.Entry(sess, textvariable=self.test_var, width=18)\
            .grid(row=1, column=1, sticky="ew", pady=2)
        ttk.Label(sess, text="Notes (jointes au CSV, à saisir avant la mesure) :")\
            .grid(row=2, column=0, columnspan=2, sticky="w", pady=(6, 0))
        self.notes = tk.Text(sess, width=30, height=4, wrap="word", font=("Consolas", 8))
        self.notes.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(2, 0))
        sess.columnconfigure(1, weight=1)

        methodf = ttk.LabelFrame(left, text="Méthode d'analyse", padding=8)
        methodf.pack(fill="x", pady=(0, 8))
        self.method_var = tk.StringVar(value=METHOD_LABELS[0][0])
        ttk.Combobox(methodf, textvariable=self.method_var, state="readonly", width=28,
                     values=[m[0] for m in METHOD_LABELS]).pack(fill="x", pady=2)
        self.method_var.trace_add("write", lambda *_: self._on_method_change())

        # Un sous-cadre de paramètres par méthode ; seul l'actif est affiché.
        self.paramf = ttk.LabelFrame(left, text="Paramètres", padding=8)
        self.paramf.pack(fill="x", pady=(0, 8))
        self.entries = {}          # {method_id: {key: StringVar}}
        self.method_frames = {}    # {method_id: Frame}
        self._build_cv_params()
        self._build_ca_params()
        self._build_dpv_params()
        self.lbl_duration = ttk.Label(self.paramf, text="", foreground="#555")
        self.lbl_duration.pack(anchor="w", pady=(4, 0))
        self._on_method_change()

        rtiaf = ttk.LabelFrame(left, text="Gamme de courant (RTIA)", padding=8)
        rtiaf.pack(fill="x", pady=(0, 8))
        ttk.Combobox(rtiaf, textvariable=self.rtia_var, state="readonly", width=20,
                     values=[RTIA_AUTO_LABEL] + [_rtia_label(o) for o in RTIA_VALUES])\
            .pack(fill="x", pady=2)
        self.lbl_rtia = ttk.Label(rtiaf, text="RTIA : —", foreground="#555")
        self.lbl_rtia.pack(anchor="w", pady=(2, 0))

        actions = ttk.LabelFrame(left, text="Mesure", padding=8)
        actions.pack(fill="x", pady=(0, 8))
        self.btn_run = ttk.Button(actions, text="▶  Lancer la mesure",
                                  command=self._run, state="disabled")
        self.btn_run.pack(fill="x", pady=2)
        self.btn_save = ttk.Button(actions, text="💾  Enregistrer CSV…",
                                   command=self._save_csv, state="disabled")
        self.btn_save.pack(fill="x", pady=2)
        self.autosave_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(actions, text="Enregistrement auto après mesure",
                        variable=self.autosave_var).pack(anchor="w", pady=(4, 0))

        logf = ttk.LabelFrame(left, text="Journal", padding=4)
        logf.pack(fill="both", expand=True)
        self.log = tk.Text(logf, width=36, height=6, state="disabled", wrap="word",
                           font=("Consolas", 8))
        self.log.pack(fill="both", expand=True)

        right = ttk.Frame(root)
        right.grid(row=0, column=1, sticky="nsew")
        right.rowconfigure(0, weight=1)
        right.columnconfigure(0, weight=1)
        self.fig = Figure(figsize=(6, 5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel("Tension appliquée (mV)")
        self.ax.set_ylabel("Courant (µA)")
        self.ax.set_title("Voltampérométrie cyclique")
        self.ax.grid(True, alpha=0.3)
        (self.curve,) = self.ax.plot([], [], "-", lw=1.2, color="#2E7D6F")
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        NavigationToolbar2Tk(self.canvas, right, pack_toolbar=False).grid(row=1, column=0, sticky="ew")

        self.status = tk.StringVar(value="Déconnecté.")
        ttk.Label(self, textvariable=self.status, relief="sunken", anchor="w", padding=4)\
            .pack(fill="x", side="bottom")

    # ── Cadres de paramètres par méthode ─────────────────────────────────────────
    def _rate_from_defaults(self):
        span = abs(DEFAULTS["peak_mv"] - DEFAULTS["start_mv"])
        dur_s = DEFAULTS["duration_ms"] / 1000.0
        return round(2.0 * span / dur_s, 1) if dur_s > 0 else 0.0

    def _add_param_row(self, frame, entries, key, label, default):
        r = len(entries)
        ttk.Label(frame, text=label).grid(row=r, column=0, sticky="w", pady=2)
        var = tk.StringVar(value=str(default))
        ttk.Entry(frame, textvariable=var, width=12, justify="right")\
            .grid(row=r, column=1, sticky="e", pady=2)
        frame.columnconfigure(0, weight=1)
        entries[key] = var
        var.trace_add("write", lambda *_: self._update_duration_label())

    def _build_cv_params(self):
        f, e = ttk.Frame(self.paramf), {}
        self._add_param_row(f, e, "start_mv", "Tension départ (mV)", DEFAULTS["start_mv"])
        self._add_param_row(f, e, "peak_mv", "Tension sommet (mV)", DEFAULTS["peak_mv"])
        self._add_param_row(f, e, "vzero_mv", "Vzero / biais (mV)", DEFAULTS["vzero_mv"])
        self._add_param_row(f, e, "steps", "Nombre de pas (pair)", DEFAULTS["steps"])
        self._add_param_row(f, e, "scan_rate", "Vitesse (mV/s)", self._rate_from_defaults())
        self._add_param_row(f, e, "sample_delay_ms", "Délai échantillon (ms)", DEFAULTS["sample_delay_ms"])
        self.entries[METHOD_CV], self.method_frames[METHOD_CV] = e, f

    def _build_ca_params(self):
        f, e = ttk.Frame(self.paramf), {}
        self._add_param_row(f, e, "pot_mv", "Potentiel (mV)", CA_DEFAULTS["pot_mv"])
        self._add_param_row(f, e, "vzero_mv", "Vzero / biais (mV)", CA_DEFAULTS["vzero_mv"])
        self._add_param_row(f, e, "duration_ms", "Durée (ms)", CA_DEFAULTS["duration_ms"])
        self._add_param_row(f, e, "interval_ms", "Intervalle (ms)", CA_DEFAULTS["interval_ms"])
        self.entries[METHOD_CA], self.method_frames[METHOD_CA] = e, f

    def _build_dpv_params(self):
        f, e = ttk.Frame(self.paramf), {}
        self._add_param_row(f, e, "start_mv", "Tension départ (mV)", DPV_DEFAULTS["start_mv"])
        self._add_param_row(f, e, "end_mv", "Tension fin (mV)", DPV_DEFAULTS["end_mv"])
        self._add_param_row(f, e, "estep_mv", "Hauteur de marche (mV)", DPV_DEFAULTS["estep_mv"])
        self._add_param_row(f, e, "epulse_mv", "Amplitude impulsion (mV)", DPV_DEFAULTS["epulse_mv"])
        self._add_param_row(f, e, "tpulse_ms", "Durée impulsion (ms)", DPV_DEFAULTS["tpulse_ms"])
        self._add_param_row(f, e, "tperiod_ms", "Période/marche (ms)", DPV_DEFAULTS["tperiod_ms"])
        self._add_param_row(f, e, "vzero_mv", "Vzero / biais (mV)", DPV_DEFAULTS["vzero_mv"])
        self.entries[METHOD_DPV], self.method_frames[METHOD_DPV] = e, f

    def _current_method_id(self):
        lbl = self.method_var.get()
        for name, mid in METHOD_LABELS:
            if name == lbl:
                return mid
        return METHOD_CV

    def _method_name(self, mid):
        for name, m in METHOD_LABELS:
            if m == mid:
                return name
        return str(mid)

    def _on_method_change(self):
        mid = self._current_method_id()
        for k, fr in self.method_frames.items():
            fr.pack(fill="x") if k == mid else fr.pack_forget()
        self._update_duration_label()

    # ── Lecture des paramètres → payload SMP + méta d'affichage ──────────────────
    def _read_params(self):
        mid = self._current_method_id()
        e = self.entries[mid]

        def fnum(key):
            try:
                return float(e[key].get())
            except ValueError:
                raise ValueError("Champs numériques invalides.")

        rtia = RTIA_LABEL_TO_OHM.get(self.rtia_var.get(), 0)

        if mid == METHOD_CV:
            start, peak, vzero = fnum("start_mv"), fnum("peak_mv"), fnum("vzero_mv")
            steps, rate, delay = int(fnum("steps")), fnum("scan_rate"), fnum("sample_delay_ms")
            if steps < 2 or steps % 2 != 0:
                raise ValueError("Le nombre de pas doit être pair et ≥ 2.")
            if rate <= 0:
                raise ValueError("La vitesse doit être > 0.")
            if peak == start:
                raise ValueError("Départ et sommet doivent différer.")
            dur_ms = int(round(2.0 * abs(peak - start) / rate * 1000.0))
            payload = {"method": METHOD_CV, "start_mv": int(round(start)),
                       "peak_mv": int(round(peak)), "vzero_mv": int(round(vzero)),
                       "steps": steps, "dur_ms": dur_ms,
                       "delay_us": int(round(delay * 1000.0)), "rtia": rtia}
            meta = {"dur_ms": dur_ms, "xlabel": "Tension appliquée (mV)",
                    "ylabel": "Courant (µA)", "xdiv": 1.0, "xcol": "voltage_mV",
                    "title": "Voltampérométrie cyclique"}
            return payload, meta

        if mid == METHOD_CA:
            pot, vzero = fnum("pot_mv"), fnum("vzero_mv")
            dur, interval = int(fnum("duration_ms")), int(fnum("interval_ms"))
            if dur < 100:
                raise ValueError("La durée doit être ≥ 100 ms.")
            if interval < 5 or interval > dur:
                raise ValueError("Intervalle : 5 ms … durée.")
            payload = {"method": METHOD_CA, "ca_pot_mv": int(round(pot)),
                       "vzero_mv": int(round(vzero)), "ca_dur_ms": dur,
                       "ca_interval_ms": interval, "rtia": rtia or 4000}
            meta = {"dur_ms": dur, "xlabel": "Temps (s)", "ylabel": "Courant (µA)",
                    "xdiv": 1000.0, "xcol": "time_ms", "title": "Chronoampérométrie"}
            return payload, meta

        # DPV
        start, end, estep = fnum("start_mv"), fnum("end_mv"), fnum("estep_mv")
        epulse = fnum("epulse_mv")
        tpulse, tperiod, vzero = int(fnum("tpulse_ms")), int(fnum("tperiod_ms")), fnum("vzero_mv")
        if end == start:
            raise ValueError("Départ et fin doivent différer.")
        if estep <= 0:
            raise ValueError("La hauteur de marche doit être > 0.")
        if tperiod <= tpulse:
            raise ValueError("La période doit dépasser la durée d'impulsion.")
        nsteps = int(abs(end - start) / estep) + 1
        dur_ms = nsteps * tperiod
        payload = {"method": METHOD_DPV, "start_mv": int(round(start)), "end_mv": int(round(end)),
                   "dpv_estep_mv": int(round(estep)), "dpv_epulse_mv": int(round(epulse)),
                   "dpv_tpulse_ms": tpulse, "dpv_tperiod_ms": tperiod,
                   "vzero_mv": int(round(vzero)), "rtia": rtia or 4000}
        meta = {"dur_ms": dur_ms, "xlabel": "Tension (mV)", "ylabel": "ΔCourant (µA)",
                "xdiv": 1.0, "xcol": "voltage_mV", "title": "DPV"}
        return payload, meta

    def _update_duration_label(self):
        lbl = getattr(self, "lbl_duration", None)
        if lbl is None:
            return
        try:
            _, meta = self._read_params()
            lbl.config(text=f"→ durée ≈ {meta['dur_ms'] / 1000:.1f} s")
        except (ValueError, KeyError, tk.TclError):
            lbl.config(text="→ durée : —")

    # ── Ports ───────────────────────────────────────────────────────────────────
    def _refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _autodetect(self):
        if self.busy or self.smp:
            return
        self.busy = True
        self.btn_detect.config(state="disabled")
        self._set_status("Détection en cours…")
        ports = [p.device for p in list_ports.comports()]
        threading.Thread(target=self._detect_worker, args=(ports,), daemon=True).start()

    def _detect_worker(self, ports):
        found = None
        for dev in ports:
            try:
                s = SmpSerial(dev)
                try:
                    cv_status(s)          # réponse SMP valide du groupe CV → c'est la carte
                    found = dev
                finally:
                    s.close()
            except Exception:
                continue
            if found:
                break
        self.q.put(("detect_done", found))

    # ── Connexion ───────────────────────────────────────────────────────────────
    def _toggle_connect(self):
        if self.smp:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("Port", "Sélectionnez un port.")
            return
        if self.busy:
            return
        try:
            self.smp = SmpSerial(port)
        except Exception as e:
            messagebox.showerror("Connexion", f"Impossible d'ouvrir {port} :\n{e}")
            self.smp = None
            return
        self.btn_connect.config(text="Déconnecter")
        self.btn_run.config(state="normal")
        self._set_status(f"Connecté à {port}.")
        self._log(f"Connecté à {port}")
        # Sonde de vie : lit STATUS pour confirmer que c'est bien la carte CV.
        threading.Thread(target=self._probe_worker, daemon=True).start()

    def _probe_worker(self):
        try:
            st = cv_status(self.smp)
            self.q.put(("probe_ok", st))
        except Exception as e:
            self.q.put(("probe_fail", str(e)))

    def _disconnect(self):
        if self.smp:
            self.smp.close()
            self.smp = None
        self.btn_connect.config(text="Connecter")
        self.btn_run.config(state="disabled")
        self._set_status("Déconnecté.")

    # ── Lancer une mesure ─────────────────────────────────────────────────────
    def _run(self):
        if self.busy or not self.smp:
            return
        try:
            params, meta = self._read_params()
        except ValueError as e:
            messagebox.showwarning("Paramètres", str(e))
            return
        self.busy = True
        self.points = []
        self.last_result = ""
        self.last_params = params
        self.last_meta = meta
        self.last_rtia_label = self.rtia_var.get()
        self.last_run_dt = datetime.now()
        self.btn_run.config(state="disabled", text="⏳  Mesure en cours…")
        self.btn_save.config(state="disabled")
        self._set_status(f"Démarrage — {meta['title']}…")
        threading.Thread(target=self._run_worker, args=(params, meta), daemon=True).start()

    def _run_worker(self, params, meta):
        try:
            cv_start(self.smp, params)
            self.q.put(("log", "→ START envoyé, mesure en cours…"))
            # Attend la fin : run repasse à false avec n>0.
            n = 0
            t0 = time.time()
            max_s = meta["dur_ms"] / 1000.0 + 20.0
            started = False
            while time.time() - t0 < max_s:
                time.sleep(0.4)
                st = cv_status(self.smp)
                n = int(st.get("n", 0))
                run = bool(st.get("run", False))
                self.q.put(("progress", n))
                if run:
                    started = True
                elif started and n > 0:
                    break
            # Télécharge les points par lots.
            pts = []
            off = 0
            while off < n:
                d = cv_data(self.smp, off)
                vs = list(d.get("v", []))
                cs = list(d.get("i", []))
                if not vs:
                    break
                for k in range(min(len(vs), len(cs))):
                    pts.append((off + k, float(vs[k]), float(cs[k])))
                off += len(vs)
                self.q.put(("download", (off, n)))
            st = cv_status(self.smp)
            self.q.put(("run_done", (pts, st)))
        except Exception as e:
            self.q.put(("run_err", str(e)))

    # ── Boucle GUI ──────────────────────────────────────────────────────────────
    def _poll_queue(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "log":
                    self._log(payload)
                elif kind == "detect_done":
                    self.busy = False
                    self.btn_detect.config(state="normal")
                    if payload:
                        self.port_var.set(payload)
                        self._set_status(f"Carte CV détectée sur {payload}.")
                    else:
                        self._set_status("Aucune carte CV détectée (SMP).")
                elif kind == "probe_ok":
                    self._set_status("Carte CV connectée (SMP OK).")
                    self._log("STATUS: " + str(payload))
                    self._apply_rtia(payload.get("rtia", 0))
                elif kind == "probe_fail":
                    self._set_status("Connecté, mais pas de réponse SMP CV — mauvais port ?")
                    self._log("⚠ pas de réponse SMP: " + payload)
                elif kind == "progress":
                    self._set_status(f"Mesure en cours… {payload} points")
                elif kind == "download":
                    off, n = payload
                    self._set_status(f"Téléchargement… {off}/{n}")
                elif kind == "run_done":
                    self._finish_run(*payload)
                elif kind == "run_err":
                    self.busy = False
                    self.btn_run.config(state="normal", text="▶  Lancer la mesure")
                    self._set_status("Échec mesure.")
                    self._log("⚠ " + payload)
                    messagebox.showerror("Mesure", payload)
        except queue.Empty:
            pass
        self.after(60, self._poll_queue)

    def _apply_rtia(self, ohm):
        try:
            ohm = int(ohm)
            self.lbl_rtia.config(text=f"RTIA actif : {ohm} Ω" if ohm else "RTIA : auto-range")
        except (ValueError, TypeError):
            pass

    def _finish_run(self, pts, st):
        self.busy = False
        self.points = pts
        self.last_status = st or {}
        self.btn_run.config(state="normal", text="▶  Lancer la mesure")
        if pts:
            self.btn_save.config(state="normal")
        peak = st.get("peak_na", 0)
        r = st.get("r_ohm", 0)
        self.last_result = f"peak_nA={peak},R_ohm={r}"
        self._apply_rtia(st.get("rtia", 0))
        self._set_status(f"Terminé — {len(pts)} points. {self.last_result}")
        self._log(f"RESULT: {self.last_result}  RTIA={st.get('rtia', 0)} Ω")
        self._redraw()
        if self.autosave_var.get() and pts:
            self._save_csv(auto=True)

    # ── Tracé ────────────────────────────────────────────────────────────────
    def _redraw(self):
        meta = self.last_meta or {}
        xdiv = meta.get("xdiv", 1.0)
        if not self.points:
            self.curve.set_data([], [])
        else:
            xs = [x / xdiv for (_, x, _) in self.points]      # ms→s en CA
            ys = [c / 1000.0 for (_, _, c) in self.points]    # nA → µA
            self.curve.set_data(xs, ys)
            self.ax.relim()
            self.ax.autoscale_view()
        self.ax.set_xlabel(meta.get("xlabel", "Tension appliquée (mV)"))
        self.ax.set_ylabel(meta.get("ylabel", "Courant (µA)"))
        self.ax.set_title(meta.get("title", "Voltampérométrie cyclique"))
        self.canvas.draw_idle()

    # ── CSV + métadonnées ───────────────────────────────────────────────────────
    def _save_csv(self, auto=False):
        if not self.points:
            if not auto:
                messagebox.showinfo("Enregistrer", "Aucune donnée.")
            return

        session = _safe_name(self.session_var.get(), "SansSession")
        test = _safe_name(self.test_var.get(), "test")
        dt = self.last_run_dt or datetime.now()
        stamp = dt.strftime("%Y%m%d_%H%M%S")
        base = f"{test}_{stamp}.csv"
        # Hiérarchie : mesures/<session>/<test>/
        folder = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "mesures", session, test)

        if auto:
            try:
                os.makedirs(folder, exist_ok=True)
            except OSError as e:
                self._log(f"⚠ Dossier non créé : {e}")
                return
            path = os.path.join(folder, base)
        else:
            try:
                os.makedirs(folder, exist_ok=True)
            except OSError:
                folder = None
            path = filedialog.asksaveasfilename(
                defaultextension=".csv", initialfile=base, initialdir=folder or None,
                filetypes=[("CSV", "*.csv"), ("Tous", "*.*")])
            if not path:
                return

        meta = self.last_meta or {}
        xcol = meta.get("xcol", "voltage_mV")
        method_name = self._method_name((self.last_params or {}).get("method", 0))
        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow([f"# session={session} test={test} "
                            f"method={method_name} date={dt:%Y-%m-%d %H:%M:%S}"])
                if self.last_result:
                    w.writerow(["# " + self.last_result])
                w.writerow(["index", xcol, "current_nA", "current_uA"])
                for idx, x, cur_na in self.points:
                    w.writerow([idx, f"{x:.1f}", f"{cur_na:.1f}", f"{cur_na / 1000.0:.4f}"])
        except OSError as e:
            messagebox.showerror("Enregistrer", f"Échec :\n{e}")
            return

        txt_path = os.path.splitext(path)[0] + ".txt"
        try:
            self._write_metadata(txt_path, session, test, dt)
        except OSError as e:
            self._log(f"⚠ Notes non enregistrées : {e}")

        self._log(f"Enregistré : {path}")
        self._set_status(f"Enregistré : {os.path.basename(path)} (+ .txt)")

    def _write_metadata(self, txt_path, session, test, dt):
        """Écrit le .txt jumeau : méthode, session/test, date, paramètres, résultat, notes."""
        p = self.last_params or {}
        st = self.last_status or {}
        notes = self.notes.get("1.0", "end").strip()
        method_id = p.get("method", 0)

        lines = [
            "Mesure électrochimique — fiche",
            "=" * 44,
            f"Session : {session}",
            f"Test    : {test}",
            f"Date    : {dt:%Y-%m-%d %H:%M:%S}",
            f"Méthode : {self._method_name(method_id)}",
            "",
            "Paramètres :",
        ]
        for key, val in p.items():
            if key == "method":
                continue
            label, unit = KEY_LABELS.get(key, (key, ""))
            lines.append(f"  {label:20}: {val} {unit}".rstrip())
        lines.append(f"  {'Gamme RTIA (libellé)':20}: {self.last_rtia_label or '—'}")

        lines += [
            "",
            "Résultat :",
            f"  {'Points collectés':20}: {len(self.points)}",
            f"  {'Pic':20}: {st.get('peak_na', '—')} nA",
            f"  {'RTIA actif':20}: {st.get('rtia', '—')} Ω",
        ]
        if method_id == METHOD_CV:
            lines.append(f"  {'Résistance estimée':20}: {st.get('r_ohm', '—')} Ω")
        lines += ["", "Notes :", notes if notes else "(aucune)", ""]

        with open(txt_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))

    # ── Utilitaires ─────────────────────────────────────────────────────────────
    def _log(self, msg):
        self.log.config(state="normal")
        self.log.insert("end", time.strftime("%H:%M:%S ") + str(msg) + "\n")
        if int(self.log.index("end-1c").split(".")[0]) > 500:
            self.log.delete("1.0", "200.0")
        self.log.see("end")
        self.log.config(state="disabled")

    def _set_status(self, msg):
        self.status.set(msg)

    def _on_close(self):
        self._disconnect()
        self.destroy()


if __name__ == "__main__":
    CVApp().mainloop()
