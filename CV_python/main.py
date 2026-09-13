# -*- coding: utf-8 -*-
"""
ParisBioTek - Pilote de voltampérométrie cyclique (CV) — transport SMP/mcumgr
=============================================================================

Pilote la carte nRF52840 + AD5940 par liaison série USB, EN PARTAGEANT le canal
avec le DFU mcumgr : les commandes CV passent par un groupe SMP custom (ID 64),
donc pas de conflit avec le DFU (même port, un seul propriétaire à la fois).

Flux :
  - START  (groupe 64, cmd 0, write) : règle les paramètres, réserve un run_id et lance.
  - STATUS (groupe 64, cmd 1, read)  : état, résultats et diagnostics détaillés.
  - DATA   (groupe 64, cmd 2, read)  : {run_id, off} -> points du même run par lots.
  - RELEASE(groupe 64, cmd 3, write) : libère le buffer après téléchargement.
  - STOP   (groupe 64, cmd 4, write) : demande l'arrêt propre du run actif.

Dépendances :  pip install pyserial cbor2 matplotlib
(tkinter est fourni avec Python sous Windows.)

Le port à choisir est CELUI du DFU/mcumgr (le même que flash.ps1). Ferme toute
autre appli qui le tient (mcumgr-client, terminal série) avant de connecter.
"""

import csv
import json
import math
import os
import re
import queue
import struct
import base64
import threading
import time
import traceback
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
CV_CMD_START, CV_CMD_STATUS, CV_CMD_DATA, CV_CMD_RELEASE, CV_CMD_STOP = 0, 1, 2, 3, 4

_HDR_PKT = b"\x06\x09"     # marqueur 1er fragment
_HDR_FRAG = b"\x04\x14"    # marqueur fragment suivant
# 90 octets bruts donnent une ligne base64 de 123 octets, compatible avec les
# firmwares déjà flashés dont le buffer RX UART ne fait que 128 octets.
_SMP_SERIAL_RAW_FRAGMENT = 90


def crc16_ccitt(data: bytes) -> int:
    """CRC16-ITU-T / XMODEM : poly 0x1021, init 0x0000, MSB d'abord (= firmware)."""
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


class SmpError(Exception):
    """Erreur enrichie du contexte du dernier échange SMP."""

    def __init__(self, message, context=None):
        super().__init__(message)
        self.context = context or {}


class SmpProtocolError(SmpError):
    pass


class SmpSerial:
    """Client SMP minimal : requête/réponse framées base64 + CRC sur le port série."""

    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.4, write_timeout=2.0)
        self.port = port
        self.baud = baud
        self.seq = 0
        self._request_lock = threading.Lock()
        self.stats = {
            "requests": 0, "responses": 0, "timeouts": 0,
            "write_timeouts": 0, "write_errors": 0, "read_errors": 0,
            "short_writes": 0,
            "lines": 0, "noise_lines": 0, "base64_errors": 0,
            "orphan_fragments": 0, "short_frames": 0, "crc_errors": 0,
            "foreign_responses": 0, "protocol_errors": 0,
            "tx_attempted_bytes": 0, "tx_bytes": 0, "rx_packet_bytes": 0,
        }
        self.last_exchange = {}

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    @staticmethod
    def _frame(packet: bytes) -> bytes:
        crc = crc16_ccitt(packet)
        body = struct.pack(">H", len(packet) + 2) + packet + struct.pack(">H", crc)
        frames = []
        for offset in range(0, len(body), _SMP_SERIAL_RAW_FRAGMENT):
            header = _HDR_PKT if offset == 0 else _HDR_FRAG
            frames.append(header + base64.b64encode(body[offset:offset + _SMP_SERIAL_RAW_FRAGMENT]) + b"\n")
        return b"".join(frames)

    def _read_packet(self, timeout: float) -> bytes:
        deadline = time.monotonic() + timeout
        buf = b""
        expected = None
        while time.monotonic() < deadline:
            try:
                line = self.ser.readline()
            except (serial.SerialException, OSError) as exc:
                self.stats["read_errors"] += 1
                self.last_exchange["error_stage"] = "serial/read"
                self.last_exchange["error"] = f"{type(exc).__name__}: {exc}"
                self.last_exchange["serial_state"] = self._serial_state()
                raise SmpError(
                    f"lecture série impossible sur {self.port}: "
                    f"{type(exc).__name__}: {exc}",
                    self.diagnostics_snapshot(),
                ) from exc
            if not line:
                continue
            self.stats["lines"] += 1
            line = line.strip(b"\r\n")
            if line.startswith(_HDR_PKT):
                try:
                    dec = base64.b64decode(line[2:], validate=True)
                except Exception:
                    self.stats["base64_errors"] += 1
                    continue
                if len(dec) < 2:
                    self.stats["short_frames"] += 1
                    continue
                expected = struct.unpack(">H", dec[:2])[0]
                buf = dec[2:]
            elif line.startswith(_HDR_FRAG):
                if expected is None:
                    self.stats["orphan_fragments"] += 1
                    continue
                try:
                    buf += base64.b64decode(line[2:], validate=True)
                except Exception:
                    self.stats["base64_errors"] += 1
                    continue
            else:
                self.stats["noise_lines"] += 1
                continue  # ligne non-SMP (bruit) : ignorée
            if expected is not None and len(buf) >= expected:
                if expected < 2:
                    self.stats["short_frames"] += 1
                    raise SmpProtocolError(f"longueur de trame SMP invalide ({expected})")
                if len(buf) != expected:
                    self.stats["protocol_errors"] += 1
                    raise SmpProtocolError(
                        f"trame SMP trop longue ({len(buf)} octets reçus, {expected} annoncés)"
                    )
                data = buf[:expected]
                packet, crc_rx = data[:-2], struct.unpack(">H", data[-2:])[0]
                crc_expected = crc16_ccitt(packet)
                if crc_expected != crc_rx:
                    self.stats["crc_errors"] += 1
                    raise SmpProtocolError(
                        f"CRC SMP invalide (reçu=0x{crc_rx:04X}, attendu=0x{crc_expected:04X})"
                    )
                self.stats["rx_packet_bytes"] += len(packet)
                return packet
        self.stats["timeouts"] += 1
        raise SmpError("pas de réponse SMP (timeout)")

    def request(self, op, group, cmd_id, payload: dict, timeout=4.0) -> dict:
        # La sonde STATUS au branchement et le bouton Mesure peuvent autrement
        # lire/réinitialiser le même port série simultanément.
        with self._request_lock:
            return self._request_locked(op, group, cmd_id, payload, timeout)

    def _serial_state(self):
        """État pyserial lisible même si le pilote Windows vient de décrocher."""
        state = {"is_open": bool(getattr(self.ser, "is_open", False))}
        for name in ("in_waiting", "out_waiting"):
            try:
                state[name] = int(getattr(self.ser, name))
            except Exception as exc:
                state[name] = f"{type(exc).__name__}: {exc}"
        return state

    def _serial_write_failure(self, exc, *, group, cmd_id, seq, started,
                              stage="serial/write"):
        timed_out = isinstance(exc, serial.SerialTimeoutException)
        counter = "write_timeouts" if timed_out else "write_errors"
        self.stats[counter] += 1
        self.last_exchange["error_stage"] = stage
        self.last_exchange["error"] = f"{type(exc).__name__}: {exc}"
        self.last_exchange["elapsed_ms"] = round(
            (time.monotonic() - started) * 1000, 1
        )
        self.last_exchange["serial_state"] = self._serial_state()
        if stage == "serial/reset_input":
            nature = "réinitialisation du buffer d'entrée impossible"
        else:
            nature = "délai d'écriture expiré" if timed_out else "écriture impossible"
        return SmpError(
            f"{nature} sur {self.port} [groupe={group}, cmd={cmd_id}, seq={seq}]. "
            "Le périphérique USB n'accepte plus les données (carte figée ou "
            "déconnectée, horloge USB interrompue, câble ou pilote Windows).",
            self.diagnostics_snapshot(),
        )

    def _request_locked(self, op, group, cmd_id, payload: dict, timeout=4.0) -> dict:
        cbor_payload = cbor2.dumps(payload)
        seq = self.seq
        self.seq = (self.seq + 1) & 0xFF
        hdr = struct.pack(">BBHHBB", op, 0, len(cbor_payload), group, seq, cmd_id)
        packet = hdr + cbor_payload
        frame = self._frame(packet)
        started = time.monotonic()
        self.stats["requests"] += 1
        self.stats["tx_attempted_bytes"] += len(frame)
        self.last_exchange = {
            "request": {"op": op, "group": group, "cmd": cmd_id, "seq": seq,
                        "payload_keys": sorted(payload), "packet_bytes": len(packet)},
            "started_monotonic": started,
        }
        try:
            self.ser.reset_input_buffer()
        except (serial.SerialException, OSError) as exc:
            raise self._serial_write_failure(
                exc, group=group, cmd_id=cmd_id, seq=seq, started=started,
                stage="serial/reset_input",
            ) from exc
        try:
            written = self.ser.write(frame)
        except (serial.SerialException, OSError) as exc:
            raise self._serial_write_failure(
                exc, group=group, cmd_id=cmd_id, seq=seq, started=started
            ) from exc
        if written != len(frame):
            self.stats["short_writes"] += 1
            exc = serial.SerialTimeoutException(
                f"écriture partielle ({written}/{len(frame)} octets)"
            )
            raise self._serial_write_failure(
                exc, group=group, cmd_id=cmd_id, seq=seq, started=started
            ) from exc
        self.stats["tx_bytes"] += written
        # Lit jusqu'à la réponse du bon groupe/commande.
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                rsp = self._read_packet(max(0.2, deadline - time.monotonic()))
            except SmpError as exc:
                self.last_exchange["error"] = str(exc)
                self.last_exchange["elapsed_ms"] = round((time.monotonic() - started) * 1000, 1)
                error_type = SmpProtocolError if isinstance(exc, SmpProtocolError) else SmpError
                raise error_type(
                    f"{exc} [groupe={group}, cmd={cmd_id}, seq={seq}]",
                    self.diagnostics_snapshot(),
                ) from exc
            if len(rsp) < 8:
                self.stats["short_frames"] += 1
                raise SmpProtocolError(
                    f"réponse SMP trop courte ({len(rsp)} octets) "
                    f"[groupe={group}, cmd={cmd_id}, seq={seq}]",
                    self.diagnostics_snapshot(),
                )
            r_op, _flags, r_len, r_group, r_seq, r_id = struct.unpack(">BBHHBB", rsp[:8])
            if r_len != len(rsp) - 8:
                self.stats["protocol_errors"] += 1
                raise SmpProtocolError(
                    f"longueur payload SMP incohérente (annoncée={r_len}, réelle={len(rsp) - 8})",
                    self.diagnostics_snapshot(),
                )
            if r_group != group or r_id != cmd_id or r_seq != seq:
                self.stats["foreign_responses"] += 1
                continue  # pas notre réponse (autre groupe = ex. DFU) : on ignore
            if r_op != op + 1:
                self.stats["protocol_errors"] += 1
                raise SmpProtocolError(
                    f"opération SMP inattendue (reçue={r_op}, attendue={op + 1})",
                    self.diagnostics_snapshot(),
                )
            body = rsp[8:]
            try:
                obj = cbor2.loads(body) if body else {}
            except Exception as exc:
                self.stats["protocol_errors"] += 1
                raise SmpProtocolError(
                    f"payload CBOR invalide: {exc}", self.diagnostics_snapshot()
                ) from exc
            if not isinstance(obj, dict):
                self.stats["protocol_errors"] += 1
                raise SmpProtocolError(
                    f"réponse CBOR inattendue ({type(obj).__name__}, map attendue)",
                    self.diagnostics_snapshot(),
                )
            if isinstance(obj, dict) and obj.get("rc", 0) not in (0, None):
                raise SmpError(
                    f"erreur SMP rc={obj.get('rc')} [groupe={group}, cmd={cmd_id}, seq={seq}]",
                    self.diagnostics_snapshot(),
                )
            nested_err = obj.get("err")
            if isinstance(nested_err, dict) and nested_err.get("rc", 0) not in (0, None):
                raise SmpError(
                    "erreur SMP "
                    f"groupe={nested_err.get('group', group)}, rc={nested_err.get('rc')} "
                    f"[cmd={cmd_id}, seq={seq}]",
                    self.diagnostics_snapshot(),
                )
            self.stats["responses"] += 1
            self.last_exchange["response"] = {
                "op": r_op, "group": r_group, "cmd": r_id, "seq": r_seq,
                "payload_keys": sorted(str(k) for k in obj), "packet_bytes": len(rsp),
            }
            # Conserver les scalaires utiles et les tailles de listes sans
            # recopier des lots complets dans chaque ligne de diagnostic.
            self.last_exchange["response"]["summary"] = {
                key: (len(value) if isinstance(value, list) else value)
                for key, value in obj.items()
                if key in ("run_id", "off", "n", "run", "err", "stage", "rc", "v", "i")
                and isinstance(value, (int, bool, str, list, type(None)))
            }
            self.last_exchange["elapsed_ms"] = round((time.monotonic() - started) * 1000, 1)
            return obj
        self.stats["timeouts"] += 1
        raise SmpError(
            f"pas de réponse SMP attendue (timeout) [groupe={group}, cmd={cmd_id}, seq={seq}]",
            self.diagnostics_snapshot(),
        )

    def diagnostics_snapshot(self):
        return {
            "port": self.port,
            "baud": self.baud,
            "serial_state": self._serial_state(),
            "stats": dict(self.stats),
            "last_exchange": dict(self.last_exchange),
        }


# ── Commandes CV ──────────────────────────────────────────────────────────────
def cv_start(smp, params):
    return smp.request(SMP_OP_WRITE, CV_GROUP, CV_CMD_START, params)


def cv_status(smp):
    return smp.request(SMP_OP_READ, CV_GROUP, CV_CMD_STATUS, {})


def cv_data(smp, off, run_id=None):
    payload = {"off": off}
    if run_id is not None:
        payload["run_id"] = run_id
    return smp.request(SMP_OP_READ, CV_GROUP, CV_CMD_DATA, payload)


def cv_release(smp, run_id, n):
    """Libère le buffer associé au run une fois DATA et STATUS validés."""
    return smp.request(
        SMP_OP_WRITE, CV_GROUP, CV_CMD_RELEASE, {"run_id": run_id, "n": n}
    )


def cv_stop(smp, run_id):
    """Demande au firmware v6+ d'arrêter proprement le run actif."""
    return smp.request(
        SMP_OP_WRITE, CV_GROUP, CV_CMD_STOP, {"run_id": run_id}
    )


# Les images récentes exposent aussi ``fw_build`` et ``diag_ver``. Ces trois
# champs restent la signature fonctionnelle du protocole multi-cycle : ils sont
# présents dans START et STATUS, tandis qu'une image plus ancienne les omet.
CV_CYCLE_PROTOCOL_FIELDS = ("steps", "cycles", "total_points")


def _as_int(value, default=0):
    """Convertit une valeur SMP en entier sans masquer une réponse incomplète."""
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _required_int(mapping, key, *, minimum=None, maximum=None):
    if key not in mapping or isinstance(mapping[key], bool) or not isinstance(mapping[key], int):
        raise SmpProtocolError(f"champ SMP '{key}' absent ou non entier")
    value = mapping[key]
    if minimum is not None and value < minimum:
        raise SmpProtocolError(f"champ SMP '{key}' hors plage ({value} < {minimum})")
    if maximum is not None and value > maximum:
        raise SmpProtocolError(f"champ SMP '{key}' hors plage ({value} > {maximum})")
    return value


def _validate_start_reply(reply):
    """Valide START et impose le bail de données annoncé par les firmwares v4+."""
    if not isinstance(reply, dict):
        raise SmpProtocolError(
            f"START invalide ({type(reply).__name__}, map attendue)"
        )

    diag_ver = None
    if "diag_ver" in reply:
        diag_ver = _required_int(reply, "diag_ver", minimum=0)

    if diag_ver is not None and diag_ver >= 4:
        _required_int(reply, "run_id", minimum=1)
        if reply.get("release_required") is not True:
            raise SmpProtocolError(
                "START diagnostic v4+ doit fournir release_required=true"
            )
        if diag_ver >= 6:
            _required_int(reply, "settle_ms", minimum=0, maximum=60000)
    else:
        # Compatibilité avec les anciennes images : contrôler tout champ
        # optionnel présent, sans inventer un bail absent de leur protocole.
        if "run_id" in reply:
            _required_int(reply, "run_id", minimum=0)
        if ("release_required" in reply and
                type(reply["release_required"]) is not bool):
            raise SmpProtocolError(
                "champ START 'release_required' présent mais non booléen"
            )
    return reply


def _validate_status(st, *, previous_n=None, previous_run_id=None, expected_points=None):
    """Valide le contrat minimal STATUS sans convertir une réponse cassée en fin de run."""
    if not isinstance(st, dict):
        raise SmpProtocolError(f"STATUS invalide ({type(st).__name__}, map attendue)")
    if "run" not in st or type(st["run"]) is not bool:
        raise SmpProtocolError("champ SMP 'run' absent ou non booléen")
    n = _required_int(st, "n", minimum=0)
    _required_int(st, "err")
    if "diag_ver" in st:
        diag_ver = _required_int(st, "diag_ver", minimum=0)
        if diag_ver >= 4:
            _required_int(st, "run_id", minimum=0)
            if "data_held" not in st or type(st["data_held"]) is not bool:
                raise SmpProtocolError(
                    "champ SMP 'data_held' absent ou non booléen dans STATUS v4+"
                )
            _required_int(st, "adiid", minimum=0, maximum=0xFFFF)
            _required_int(st, "chipid", minimum=0, maximum=0xFFFF)
            _required_int(st, "spi_mode", minimum=-1, maximum=3)
            _required_int(st, "mode_ids01", minimum=0, maximum=0xFFFFFFFF)
            _required_int(st, "mode_ids23", minimum=0, maximum=0xFFFFFFFF)
            _required_int(st, "gpio_err")
            _required_int(st, "gpio_state", minimum=0, maximum=0xFFFFFFFF)
            _required_int(st, "platform_err")
            _required_int(st, "spi_errors", minimum=0)
            _required_int(st, "spi_last")
        if diag_ver >= 5:
            _required_int(st, "mode_tried", minimum=0, maximum=0x0F)
            if "hfxo_on" not in st or type(st["hfxo_on"]) is not bool:
                raise SmpProtocolError(
                    "champ SMP 'hfxo_on' absent ou non booléen dans STATUS v5+"
                )
        if diag_ver >= 6:
            if "cancelled" not in st or type(st["cancelled"]) is not bool:
                raise SmpProtocolError(
                    "champ SMP 'cancelled' absent ou non booléen dans STATUS v6+"
                )
            if "stop_requested" not in st or type(st["stop_requested"]) is not bool:
                raise SmpProtocolError(
                    "champ SMP 'stop_requested' absent ou non booléen dans STATUS v6+"
                )
            _required_int(st, "settle_ms", minimum=0, maximum=60000)
            if st["cancelled"]:
                if st["run"] or not st["stop_requested"] or st.get("stage") != "cancelled":
                    raise SmpProtocolError(
                        "STATUS v6 d'arrêt incohérent (cancelled exige run=false, "
                        "stop_requested=true et stage='cancelled')"
                    )
                if st["err"] != 0:
                    raise SmpProtocolError(
                        "STATUS v6 d'arrêt utilisateur ne doit pas déclarer d'erreur firmware"
                    )
    run_id = st.get("run_id")
    if previous_run_id is not None and run_id is None:
        raise SmpProtocolError("champ SMP 'run_id' disparu pendant la mesure")
    if run_id is not None:
        if isinstance(run_id, bool) or not isinstance(run_id, int) or run_id < 0:
            raise SmpProtocolError(f"champ SMP 'run_id' invalide ({run_id!r})")
        if previous_run_id is not None and run_id != previous_run_id:
            raise SmpProtocolError(
                f"la carte a changé de run_id pendant la mesure ({previous_run_id} → {run_id})"
            )
    if previous_n is not None and n < previous_n and (
            previous_run_id is None or run_id is None or run_id == previous_run_id):
        raise SmpProtocolError(f"le compteur de points a reculé ({previous_n} → {n})")
    if expected_points is not None and n > expected_points:
        raise SmpProtocolError(
            f"le firmware annonce {n} points, au-delà des {expected_points} attendus"
        )
    return st


def _status_timeline_entry(st, client_elapsed_ms):
    """Sous-ensemble compact mais causal d'un STATUS pour le rapport JSON."""
    return {
        "client_elapsed_ms": round(client_elapsed_ms),
        "fw_build": st.get("fw_build"), "diag_ver": st.get("diag_ver"),
        "run_id": st.get("run_id"), "run": st.get("run"),
        "data_held": st.get("data_held"), "cancelled": st.get("cancelled"),
        "stop_requested": st.get("stop_requested"),
        "settle_ms": st.get("settle_ms"),
        "stage": st.get("stage"), "error_stage": st.get("error_stage"),
        "n": st.get("n"), "err": st.get("err"),
        "err_detail": st.get("err_detail"),
        "elapsed_ms": st.get("elapsed_ms"), "idle_ms": st.get("idle_ms"),
        "lfosc_meas_hz": st.get("lfosc_meas_hz"),
        "lfosc_hz": st.get("lfosc_hz"), "lfosc_err": st.get("lfosc_err"),
        "lfosc_fallback": st.get("lfosc_fallback"),
        "platform_err": st.get("platform_err"),
        "spi_errors": st.get("spi_errors"), "spi_last": st.get("spi_last"),
        "adiid": st.get("adiid"), "chipid": st.get("chipid"),
        "spi_mode": st.get("spi_mode"),
        "mode_ids01": st.get("mode_ids01"),
        "mode_ids23": st.get("mode_ids23"),
        "mode_tried": st.get("mode_tried"), "hfxo_on": st.get("hfxo_on"),
        "gpio_err": st.get("gpio_err"), "gpio_state": st.get("gpio_state"),
        "ramp_error": st.get("ramp_error"), "int_fatal": st.get("int_fatal"),
        "intc0": st.get("intc0"), "intc1": st.get("intc1"),
        "intc0_rejected": st.get("intc0_rejected"),
        "intc1_rejected": st.get("intc1_rejected"),
        "intc_stale": st.get("intc_stale"),
        "intc_retries": st.get("intc_retries"),
        "fatal_rechecks": st.get("fatal_rechecks"),
        "fatal_unconfirmed": st.get("fatal_unconfirmed"),
        "seq_con": st.get("seq_con"), "seq_count": st.get("seq_count"),
        "seq_timeout": st.get("seq_timeout"),
        "seq0_info": st.get("seq0_info"), "seq1_info": st.get("seq1_info"),
        "seq2_info": st.get("seq2_info"), "seq3_info": st.get("seq3_info"),
        "init_endseq_missing": st.get("init_endseq_missing"),
        "init_wait_polls": st.get("init_wait_polls"),
        "init_direct_fallback": st.get("init_direct_fallback"),
        "custom_irq": st.get("custom_irq"), "refills": st.get("refills"),
    }


def _validate_data_reply(reply, requested_off, *, run_id=None, max_points=4000):
    """Retourne (n_carte, [(x, i), ...]) après validation stricte de DATA."""
    if not isinstance(reply, dict):
        raise SmpProtocolError(f"DATA invalide ({type(reply).__name__}, map attendue)")
    reply_off = _required_int(reply, "off", minimum=0)
    reply_n = _required_int(reply, "n", minimum=0)
    if reply_off != requested_off:
        raise SmpProtocolError(
            f"offset DATA incohérent (demandé={requested_off}, reçu={reply_off})"
        )
    if reply_n > max_points:
        raise SmpProtocolError(f"DATA annonce {reply_n} points (maximum {max_points})")
    if run_id is not None:
        if "run_id" not in reply:
            raise SmpProtocolError("champ SMP 'run_id' absent de DATA")
        data_run_id = _required_int(reply, "run_id", minimum=0)
        if data_run_id != run_id:
            raise SmpProtocolError(
                f"DATA appartient au run_id {data_run_id}, pas au run_id {run_id}"
            )
    vs, currents = reply.get("v"), reply.get("i")
    if not isinstance(vs, list) or not isinstance(currents, list):
        raise SmpProtocolError("DATA doit contenir deux listes 'v' et 'i'")
    if len(vs) != len(currents):
        raise SmpProtocolError(
            f"lot DATA tronqué (v={len(vs)} valeur(s), i={len(currents)} valeur(s))"
        )
    if len(vs) > 64:
        raise SmpProtocolError(f"lot DATA trop grand ({len(vs)} > 64)")
    if requested_off + len(vs) > reply_n:
        raise SmpProtocolError(
            f"lot DATA dépasse n (off={requested_off}, lot={len(vs)}, n={reply_n})"
        )
    pairs = []
    for index, (xval, current) in enumerate(zip(vs, currents)):
        if (isinstance(xval, bool) or not isinstance(xval, (int, float)) or
                isinstance(current, bool) or not isinstance(current, (int, float)) or
                not math.isfinite(float(xval)) or not math.isfinite(float(current))):
            raise SmpProtocolError(f"valeur DATA invalide à l'index {requested_off + index}")
        pairs.append((float(xval), float(current)))
    return reply_n, pairs


def _validate_release_reply(reply, run_id, n):
    if not isinstance(reply, dict):
        raise SmpProtocolError("réponse RELEASE invalide (map attendue)")
    reply_run_id = _required_int(reply, "run_id", minimum=0)
    reply_n = _required_int(reply, "n", minimum=0)
    if reply_run_id != run_id or reply_n != n:
        raise SmpProtocolError(
            f"ACK RELEASE incohérent (run_id={reply_run_id}, n={reply_n}; "
            f"attendu run_id={run_id}, n={n})"
        )
    if reply.get("released") is not True:
        raise SmpProtocolError("ACK RELEASE sans released=true")


def _validate_stop_reply(reply, run_id):
    """Valide l'ACK STOP v6 sans inventer un arrêt si le firmware est ancien."""
    if not isinstance(reply, dict):
        raise SmpProtocolError("réponse STOP invalide (map attendue)")
    rc = _required_int(reply, "rc")
    reply_run_id = _required_int(reply, "run_id", minimum=1)
    _required_int(reply, "n", minimum=0, maximum=4000)
    if rc != 0:
        raise SmpProtocolError(f"ACK STOP en erreur (rc={rc})")
    if reply_run_id != run_id:
        raise SmpProtocolError(
            f"ACK STOP incohérent (run_id={reply_run_id}; attendu={run_id})"
        )
    if type(reply.get("stop_requested")) is not bool:
        raise SmpProtocolError("ACK STOP sans stop_requested booléen")
    return reply


def _cv_cycle_protocol_missing(reply):
    """Champs de confirmation absents d'une réponse START/STATUS CV."""
    if not isinstance(reply, dict):
        return list(CV_CYCLE_PROTOCOL_FIELDS)
    return [key for key in CV_CYCLE_PROTOCOL_FIELDS if key not in reply]


def _start_ack_summary(reply, params, meta):
    """Résumé journalisable de la configuration réellement confirmée par START."""
    missing = _cv_cycle_protocol_missing(reply)
    control = (
        f"firmware={reply.get('fw_build', 'non renseigné')} ; "
        f"diagnostic=v{reply.get('diag_ver', '—')} ; "
        f"run_id={reply.get('run_id', '—')} ; "
        f"RELEASE requis={reply.get('release_required', False)}"
    )
    if params.get("method") != METHOD_CV:
        return f"START ACK reçu : {control}."
    if missing:
        return ("⚠ START ACK sans confirmation CV multi-cycle "
                f"(champs absents : {', '.join(missing)}). "
                f"Firmware possiblement antérieur à cette GUI ; {control}.")
    return ("START ACK : " + control + " ; " +
            f"{_as_int(reply.get('steps'))} pas/cycle × "
            f"{_as_int(reply.get('cycles'))} cycle(s) = "
            f"{_as_int(reply.get('total_points'))} point(s) "
            f"(attendu : {meta.get('expected_points', '—')}).")


INTERRUPT_FLAGS = (
    (0x00000200, "CUSTOMINT0"), (0x00008000, "ENDSEQ"),
    (0x00020000, "SEQTIMEOUTERR"), (0x00200000, "CMDFIFO_OVERFLOW"),
    (0x00400000, "CMDFIFO_UNDERFLOW"), (0x04000000, "DATAFIFO_OVERFLOW"),
    (0x08000000, "DATAFIFO_UNDERFLOW"),
)

# Mot de passe du registre AD5940 SEQSLPLOCK. S'il apparait tel quel dans
# INTC0, la valeur provient d'une lecture perimee pendant une transition de
# sommeil ; ce n'est pas un ensemble valide de drapeaux d'interruption.
SEQSLPLOCK_UNLOCK_SIGNATURE = 0x000A47E5
SEQTIMEOUTERR_MASK = 0x00020000


def _flag_summary(value):
    value = _as_int(value)
    if value == 0:
        return "0x00000000 (aucun)"
    if value == SEQSLPLOCK_UNLOCK_SIGNATURE:
        return "0x000A47E5 (LECTURE_PÉRIMÉE:SEQSLPLOCK_UNLOCK)"
    names = [name for mask, name in INTERRUPT_FLAGS if value & mask]
    return f"0x{value:08X}" + (f" ({'|'.join(names)})" if names else "")


def _spi_mode_ids(st):
    """Dépaquette les quatre ADIID 16 bits des diagnostics firmware v4+."""
    if not isinstance(st, dict) or "mode_ids01" not in st or "mode_ids23" not in st:
        return None
    ids01 = _as_int(st.get("mode_ids01")) & 0xFFFFFFFF
    ids23 = _as_int(st.get("mode_ids23")) & 0xFFFFFFFF
    return (ids01 & 0xFFFF, (ids01 >> 16) & 0xFFFF,
            ids23 & 0xFFFF, (ids23 >> 16) & 0xFFFF)


def _spi_mode_ids_summary(st):
    ids = _spi_mode_ids(st)
    if ids is None:
        return "non renseignés"
    diag_ver = _as_int(st.get("diag_ver")) if isinstance(st, dict) else 0
    if diag_ver >= 5 and "mode_tried" in st:
        tried_mask = _as_int(st.get("mode_tried")) & 0x0F
        return "/".join(
            (f"M{mode}=0x{adiid:04X}" if tried_mask & (1 << mode)
             else f"M{mode}=non essayé")
            for mode, adiid in enumerate(ids)
        )
    return "/".join(f"M{mode}=0x{adiid:04X}" for mode, adiid in enumerate(ids))


def _gpio_state_summary(value):
    """Décode l'instantané des commandes/latches OUT nRF avant la sonde v4."""
    if value is None:
        return "non renseigné"
    state = _as_int(value) & 0xFFFFFFFF
    return (
        f"0x{state:03X} (commandé PWR={'ON' if state & 0x1 else 'OFF'}, "
        f"CS={'actif-bas' if state & 0x2 else 'inactif'}, "
        f"RESET={'actif-bas' if state & 0x4 else 'relâché'} ; "
        f"latch PWR/CS/RESET="
        f"{'H' if state & 0x100 else 'L'}/"
        f"{'H' if state & 0x200 else 'L'}/"
        f"{'H' if state & 0x400 else 'L'})"
    )


def _status_summary(st, prefix="STATUS"):
    """Résumé lisible et complet d'un STATUS de diagnostic v2 à v6."""
    if not isinstance(st, dict):
        return f"{prefix}: réponse invalide {st!r}"
    core_keys = ("fw_build", "diag_ver", "run_id", "run", "cancelled",
                 "stop_requested", "settle_ms", "data_held", "stage",
                 "error_stage", "n", "total_points", "err", "err_detail",
                 "elapsed_ms", "idle_ms", "method", "steps", "cycles", "rtia", "rtia_cal")
    lines = [f"{prefix}: " + ", ".join(f"{key}={st.get(key, '—')}" for key in core_keys)]
    if "diag_ver" in st:
        diag_ver = _as_int(st.get("diag_ver"))
        lines.append(
            "  Horloges/SPI : "
            f"ADIID=0x{_as_int(st.get('adiid')):04X}, "
            + (f"CHIPID=0x{_as_int(st.get('chipid')):04X}, "
               if diag_ver >= 4 else "") +
            f"platform_err={st.get('platform_err', '—')}, "
            f"LFOSC mesuré/utilisé={st.get('lfosc_meas_hz', '—')}/{st.get('lfosc_hz', '—')} Hz, "
            f"lfosc_err={st.get('lfosc_err', '—')}, repli={st.get('lfosc_fallback', '—')}, "
            f"spi_errors={st.get('spi_errors', '—')}, spi_last={st.get('spi_last', '—')}"
        )
        if diag_ver >= 4:
            probe_version = 5 if diag_ver >= 5 else 4
            lines.append(
                f"  Sonde matérielle v{probe_version} : "
                f"mode retenu={st.get('spi_mode', '—')}, "
                f"ADIID par mode={_spi_mode_ids_summary(st)}, "
                + (f"modes essayés=0x{_as_int(st.get('mode_tried')) & 0x0F:X}, "
                   f"OE_OSC1_commandé={'ON' if st.get('hfxo_on') is True else 'OFF'}, "
                   if diag_ver >= 5 else "") +
                f"gpio_rc={st.get('gpio_err', '—')}, "
                f"GPIO avant sonde (latches OUT, pas niveaux électriques)="
                f"{_gpio_state_summary(st.get('gpio_state'))}"
                + ("" if diag_ver >= 5 else
                   "; 0xFFFF=non essayé ou valeur effectivement lue")
            )
        lines.append(
            "  Séquenceur : "
            f"ramp_error={st.get('ramp_error', '—')}, "
            f"INTC0={_flag_summary(st.get('intc0'))}, "
            f"INTC1={_flag_summary(st.get('intc1'))}, "
            f"fatal={_flag_summary(st.get('int_fatal'))}, "
            f"CUSTOM={st.get('custom_irq', '—')}, refills={st.get('refills', '—')}, "
            f"blocs={st.get('blocks', '—')}, pas/bloc={st.get('block_steps', '—')}, "
            f"reste={st.get('steps_remaining', '—')}, "
            f"FIFO={st.get('fifo_count', '—')} (max={st.get('fifo_max', '—')})"
        )
        if diag_ver >= 3:
            lines.append(
                "  Intégrité INTC : "
                f"lectures sommeil rejetées={st.get('intc_stale', '—')}, "
                f"retries={st.get('intc_retries', '—')}, "
                f"fatals revérifiés/non confirmés="
                f"{st.get('fatal_rechecks', '—')}/{st.get('fatal_unconfirmed', '—')}, "
                f"dernier rejet={_flag_summary(st.get('intc0_rejected'))}/"
                f"{_flag_summary(st.get('intc1_rejected'))}"
            )
            lines.append(
                "  Registres séquenceur sur faute confirmée : "
                f"SEQCON=0x{_as_int(st.get('seq_con')):08X}, "
                f"SEQCNT={_as_int(st.get('seq_count'))}, "
                f"SEQTIMEOUT={_as_int(st.get('seq_timeout'))}, "
                f"SEQ0/1/2INFO=0x{_as_int(st.get('seq0_info')):08X}/"
                f"0x{_as_int(st.get('seq1_info')):08X}/"
                f"0x{_as_int(st.get('seq2_info')):08X}"
            )
            if "init_wait_polls" in st:
                lines.append(
                    "  Initialisation sequence : "
                    f"polls={st.get('init_wait_polls', '—')}, "
                    f"ENDSEQ absent mais SEQ_STOP confirme="
                    f"{st.get('init_endseq_missing', '—')}, "
                    f"repli MCU direct={'oui' if _as_int(st.get('init_direct_fallback')) else 'non'}, "
                    f"SEQ3INFO=0x{_as_int(st.get('seq3_info')):08X}"
                )
    return "\n".join(lines)


MEASUREMENT_ERROR_LABELS = {
    -2: "buffer de points plein",
    -3: "échantillon numérique invalide",
    -10: "initialisation de la plateforme AD5940 impossible",
    -11: "identifiant AD5940 incorrect (liaison SPI/alimentation)",
    -12: "transfert SPI en erreur",
    -20: "initialisation du moteur CV impossible",
    -21: "démarrage du séquenceur CV impossible",
    -22: "service du séquenceur/FIFO en erreur",
    -23: "aucun nouveau point reçu par le firmware",
    -24: "durée maximale firmware dépassée",
    -25: "fin de séquence avec un nombre de points incomplet",
    -26: "pré-polarisation au potentiel initial impossible",
    -30: "échec de la chronoampérométrie",
    -40: "échec de la DPV",
}


def _measurement_error_message(st, params, meta, start_reply):
    """Traduit l'erreur générique du firmware en diagnostic actionnable.

    `n=0` est le nombre de points de la *vraie* CV stockés par le firmware :
    les balayages de sonde de l'auto-range ne sont volontairement pas ajoutés
    au buffer. Ce nombre ne permet donc pas, seul, de distinguer un échec
    d'initialisation AD5940 d'un échec du séquenceur/FIFO.
    """
    st = st if isinstance(st, dict) else {}
    err = _as_int(st.get("err"))
    n = _as_int(st.get("n"))
    expected = _as_int(meta.get("expected_points"))
    label = MEASUREMENT_ERROR_LABELS.get(err, "erreur firmware non documentée")
    lines = [
        f"Le firmware a interrompu la mesure : {label} ",
        f"Build={st.get('fw_build', 'non renseigné')}, diagnostic=v{st.get('diag_ver', '—')}. "
        f"Code err={err}, détail={st.get('err_detail', '—')}, "
        f"phase={st.get('stage', '—')}, phase_erreur={st.get('error_stage', '—')}, "
        f"points={n}/{expected or '—'}, "
        f"temps={st.get('elapsed_ms', '—')} ms, sans progrès={st.get('idle_ms', '—')} ms.",
    ]

    if st.get("diag_ver"):
        lines.append(
            f"Plateforme rc={st.get('platform_err', '—')}; "
            f"LFOSC mesuré/utilisé : {st.get('lfosc_meas_hz', '—')}/"
            f"{st.get('lfosc_hz', '—')} Hz (rc={st.get('lfosc_err', '—')}); "
            f"repli={'oui' if st.get('lfosc_fallback') else 'non'}; "
            f"SPI : {st.get('spi_errors', '—')} erreur(s), dernier code={st.get('spi_last', '—')}."
        )
        if _as_int(st.get("diag_ver")) >= 4:
            probe_version = 5 if _as_int(st.get("diag_ver")) >= 5 else 4
            lines.append(
                f"Sonde SPI v{probe_version} : ADIID=0x{_as_int(st.get('adiid')):04X}, "
                f"CHIPID=0x{_as_int(st.get('chipid')):04X}, "
                f"mode retenu={st.get('spi_mode', '—')}, "
                f"{_spi_mode_ids_summary(st)}; "
                + (f"masque modes essayés=0x{_as_int(st.get('mode_tried')) & 0x0F:X}; "
                   f"OE OSC1 commandé={'ON' if st.get('hfxo_on') is True else 'OFF'}; "
                   if probe_version >= 5 else "") +
                f"gpio_rc={st.get('gpio_err', '—')}; "
                "GPIO avant sonde (latches OUT nRF, pas tensions mesurées)="
                f"{_gpio_state_summary(st.get('gpio_state'))}."
            )
        lines.append(
            f"Séquenceur : ramp_error={st.get('ramp_error', '—')}, "
            f"INTC0={_flag_summary(st.get('intc0'))}, INTC1={_flag_summary(st.get('intc1'))}, "
            f"fatal={_flag_summary(st.get('int_fatal'))}, "
            f"CUSTOM/refills={st.get('custom_irq', '—')}/{st.get('refills', '—')}."
        )
        if _as_int(st.get("diag_ver")) >= 3:
            lines.append(
                "Contrôle des lectures INTC : "
                f"{st.get('intc_stale', '—')} signature(s) de sommeil rejetée(s), "
                f"{st.get('intc_retries', '—')} nouvelle(s) lecture(s), "
                f"{st.get('fatal_rechecks', '—')} faute(s) revérifiée(s), "
                f"{st.get('fatal_unconfirmed', '—')} non confirmée(s)."
            )
            lines.append(
                "Registres capturés sur faute confirmée : "
                f"SEQCON=0x{_as_int(st.get('seq_con')):08X}, "
                f"SEQCNT={_as_int(st.get('seq_count'))}, "
                f"SEQTIMEOUT={_as_int(st.get('seq_timeout'))}, "
                f"SEQ0/1/2INFO=0x{_as_int(st.get('seq0_info')):08X}/"
                f"0x{_as_int(st.get('seq1_info')):08X}/"
                f"0x{_as_int(st.get('seq2_info')):08X}."
            )
            if "init_wait_polls" in st:
                lines.append(
                    "Initialisation sequence : "
                    f"{st.get('init_wait_polls', '—')} poll(s), "
                    f"ENDSEQ absent mais SEQ_STOP confirme="
                    f"{st.get('init_endseq_missing', '—')}, "
                    f"repli MCU direct={'oui' if _as_int(st.get('init_direct_fallback')) else 'non'}, "
                    f"SEQ3INFO=0x{_as_int(st.get('seq3_info')):08X}."
                )

        if _as_int(st.get("init_direct_fallback")):
            lines.append(
                "Initialisation : le déclenchement manuel MMR de SEQ3 n'a pas "
                "démarré ; la même configuration a été appliquée directement par SPI. "
                "Les séquences d'acquisition ADC/DAC restent déclenchées par le WUPT."
            )

        if err == -20 and _as_int(st.get("err_detail")) == -10:
            lines.append(
                "Détail -10 : timeout interne AD5940 pendant cv/init. "
                "Le diagnostic Init ci-dessus indique si la séquence a réellement "
                "terminé sans ENDSEQ ou si elle est restée active."
            )

        intc0 = _as_int(st.get("intc0"))
        intc1 = _as_int(st.get("intc1"))
        if (intc0 == SEQSLPLOCK_UNLOCK_SIGNATURE and
                not (intc1 & SEQTIMEOUTERR_MASK)):
            lines.append(
                "Cause identifiée : INTC0 vaut exactement 0x000A47E5, le mot "
                "de passe SEQSLPLOCK de l'AD5940, tandis qu'INTC1 ne confirme "
                "pas SEQTIMEOUTERR. Il s'agit d'une lecture SPI périmée pendant "
                "l'hibernation, interprétée à tort comme une faute séquenceur. "
                "Le firmware de diagnostic v3 verrouille cette transition et "
                "revérifie toute faute avant d'arrêter la CV."
            )

        diag_ver = _as_int(st.get("diag_ver"))
        mode_ids = _spi_mode_ids(st)
        spi_mode = _as_int(st.get("spi_mode"), -1)
        sensor_ids_silent = (
            _as_int(st.get("adiid")) in (0x0000, 0xFFFF) and
            _as_int(st.get("chipid")) in (0x0000, 0xFFFF)
        )
        silent_modes_confirmed = False
        if mode_ids is not None:
            if diag_ver >= 5:
                tried_mask = _as_int(st.get("mode_tried")) & 0x0F
                tried_ids = [
                    value for mode, value in enumerate(mode_ids)
                    if tried_mask & (1 << mode)
                ]
                silent_modes_confirmed = (
                    bool(tried_ids) and
                    all(value in (0x0000, 0xFFFF) for value in tried_ids)
                )
            else:
                # En v4, 0xFFFF était aussi le marqueur « non essayé » : une
                # map entièrement à 0xFFFF ne prouve donc pas un capteur muet.
                silent_modes_confirmed = (
                    any(value == 0x0000 for value in mode_ids) and
                    all(value in (0x0000, 0xFFFF) for value in mode_ids)
                )
        if err == -10 and _as_int(st.get("gpio_err")) != 0:
            lines.append(
                "Cause détectée : une commande GPIO d'alimentation/CS/RESET a "
                f"échoué (rc={st.get('gpio_err')}). La mesure n'a pas atteint "
                "l'AD5940 ; il ne s'agit ni du transport USB/SMP ni des paramètres CV."
            )
        elif (err == -10 and _as_int(st.get("spi_errors")) == 0 and
              spi_mode == -1 and sensor_ids_silent and silent_modes_confirmed):
            if diag_ver >= 5 and (_as_int(st.get("mode_tried")) & 0x0F) == 0x0F:
                probe_scope = "dans les quatre modes SPI"
            else:
                probe_scope = "dans tous les modes SPI réellement essayés"
            lines.append(
                "Cause isolée : le contrôleur SPI nRF a terminé les transferts, "
                f"mais l'AD5940 n'a fourni aucun identifiant valide {probe_scope}. "
                "Ce n'est pas une panne USB/SMP, du tracé Python, du RTIA, de la "
                "vitesse ou de la fenêtre CV. Vérifier en priorité DVDD/IOVDD et "
                "AVDD pendant START, RESET physiquement haut avant la sonde, MISO "
                "et le connecteur capteur."
            )

        if err == -10 and st.get("fw_build") == "20260824-intc-v3":
            lines.append(
                "Conflit de build identifié : intc-v3 héritait du QSPI de la "
                "cible nRF52840 DK sur P0.17/P0.20/P0.22, en conflit avec "
                "CS/RESET/MOSI de l'AD5940. Le build pwr-v4 désactive ce QSPI et "
                "journalise les vraies lectures d'identité et les GPIO."
            )
    elif err == -1 and n > 0 and expected:
        # Compatibilité avec l'ancien firmware de la capture : celui-ci ne
        # transmettait que -1 après durée+6 s. Estimation uniquement indicative.
        nominal_period_ms = meta.get("dur_ms", 0) / expected if expected else 0
        old_timeout_ms = meta.get("dur_ms", 0) + 6000
        inferred_lfosc = (32000.0 * old_timeout_ms / n / nominal_period_ms
                           if nominal_period_ms > 0 else 0)
        lines.append(
            "Ce firmware ne fournit pas encore le diagnostic v2. Le rythme observé "
            f"est compatible avec une calibration LFOSC aberrante (~{inferred_lfosc:.0f} Hz "
            "au lieu d'environ 32 000 Hz), ou avec un arrêt du séquenceur."
        )

    if params.get("method") == METHOD_CV:
        method = _as_int(st.get("method"), METHOD_CV)
        if method != METHOD_CV:
            lines.append(
                "Le STATUS ne correspond pas à la méthode CV demandée ; "
                "la carte a pu redémarrer ou une autre commande a été reçue."
            )
        if n == 0:
            lines.append(
                "Aucun point de la CV demandée n'a atteint le buffer. "
                "Les éventuelles sondes d'auto-range ne sont pas comptées."
            )
            if not st.get("diag_ver") and "rtia_cal" in st and _as_int(st.get("rtia_cal")) == 0:
                lines.append(
                    "La calibration RTIA est également absente : l'échec est "
                    "probablement survenu avant l'acquisition (AD5940, alimentation "
                    "ou liaison SPI), et non dans le tracé Python."
                )
            elif not st.get("diag_ver") and "rtia_cal" in st:
                lines.append(
                    "Le RTIA a été calibré, mais aucun échantillon FIFO n'est arrivé : "
                    "vérifier le séquenceur/les interruptions AD5940."
                )
        elif expected and n < expected:
            lines.append(f"La CV a été interrompue après {n}/{expected} point(s).")

        missing = _cv_cycle_protocol_missing(start_reply)
        if missing:
            lines.append(
                "La réponse START ne confirme pas le protocole multi-cycle "
                f"(champs absents : {', '.join(missing)}). Un firmware ancien ou "
                "non flashé est possible."
            )
        if (_as_int(params.get("rtia")) == 0 and
                not _as_int(st.get("int_fatal"))):
            lines.append(
                "Pour isoler le défaut, désactivez temporairement Auto et choisissez "
                "10 kΩ : l'auto-range lance des balayages de sonde avant la CV."
            )

    return "\n".join(lines)


def _known_transport_failure_hint(exc, st, expected_points):
    """Reconnaît le gel de fin de mesure propre à l'image pwr-v4."""
    if (not isinstance(exc, SmpError) or isinstance(exc, SmpProtocolError) or
            not isinstance(st, dict)):
        return ""
    context = exc.context if isinstance(exc.context, dict) else {}
    exchange = context.get("last_exchange", {})
    error_stage = exchange.get("error_stage") if isinstance(exchange, dict) else None
    if error_stage not in ("serial/write", "serial/reset_input", "serial/read"):
        return ""
    expected = _as_int(expected_points)
    n = _as_int(st.get("n"))
    near_end = expected > 0 and 0 <= expected - n <= max(32, expected // 50)
    if (st.get("fw_build") == "20260824-pwr-v4" and st.get("run") is True and
            _as_int(st.get("err")) == 0 and near_end):
        return (
            "\nCause connue du build 20260824-pwr-v4 : à la fin de la mesure, "
            "le firmware abaisse P0.08 (OE d'OSC1, horloge 32 MHz du nRF) alors "
            "que l'USB l'utilise encore. Le CPU et le port SMP se figent avant "
            "run=false/RELEASE. Flasher le build 20260825-directinit-v8 recommandé "
            "(correctif d'horloge présent depuis 20260824-clk-v5)."
        )
    return ""


KNOWN_UNSAFE_FIRMWARE = {
    "20260824-intc-v3": (
        "cette image hérite du QSPI du nRF52840 DK sur les broches "
        "CS/RESET/MOSI de l'AD5940"
    ),
    "20260824-pwr-v4": (
        "cette image coupe P0.08 (OE d'OSC1 32 MHz) à la fin de la mesure "
        "et fige le CPU/USB avant STATUS final et RELEASE"
    ),
}


def _firmware_measurement_blocker(status):
    """Message bloquant pour les images dont l'échec est déterministe."""
    if not isinstance(status, dict):
        return ""
    build = status.get("fw_build")
    reason = KNOWN_UNSAFE_FIRMWARE.get(build)
    if not reason:
        return ""
    return (
        f"Mesure interdite avec le firmware {build} : {reason}. "
        "Flashez 20260825-directinit-v8 (diag_ver=6) ou une version plus récente."
    )


def _firmware_supports_stop(status):
    """STOP et la stabilisation confirmée font partie du contrat diagnostic v6."""
    return isinstance(status, dict) and _as_int(status.get("diag_ver"), -1) >= 6


# ─────────────────────────────────────────────────────────────────────────────
# Paramètres et gamme RTIA
# ─────────────────────────────────────────────────────────────────────────────
DEFAULTS = {
    "start_mv": -1000.0, "peak_mv": 1000.0, "vzero_mv": 1300.0,
    "steps": 800, "cycles": 1, "duration_ms": 24000, "sample_delay_ms": 7.0,
    "settle_ms": 2000,
}

# Le firmware conserve les courbes en RAM avant leur téléchargement SMP.
# 4 000 laisse la marge nécessaire au moteur de séquences (bien que son buffer
# de transfert fasse 4 096 points) ; rester synchronisé avec le firmware.
CV_MAX_POINTS = 4000

# Plage représentable par LPDAC0 : Vzero est codé sur 6 bits et Vbias doit
# rester entre 0,2 et 2,4 V. Les mêmes contrôles sont faits par le firmware.
DAC_MIN_MV = 200.0
DAC_MAX_MV = 2400.0
DAC_VZERO_MAX_MV = DAC_MIN_MV + 63.0 * (2200.0 / 4095.0 * 64.0)


def validate_potential_window(vzero_mv, *potentials_mv):
    """Refuse une consigne qui serait silencieusement écrêtée par le LPDAC."""
    if not DAC_MIN_MV <= vzero_mv <= DAC_VZERO_MAX_MV:
        raise ValueError(f"Vzero doit être entre {DAC_MIN_MV:.0f} et {DAC_VZERO_MAX_MV:.0f} mV.")
    for potential_mv in potentials_mv:
        vbias_mv = vzero_mv + potential_mv
        if not DAC_MIN_MV <= vbias_mv <= DAC_MAX_MV:
            raise ValueError(
                "Vzero + potentiel doit rester entre 200 et 2400 mV "
                f"(valeur demandée : {vbias_mv:.0f} mV)."
            )

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
    "steps": ("Nombre de pas / cycle", ""), "cycles": ("Nombre de cycles", ""),
    "dur_ms": ("Durée / cycle", "ms"),
    "settle_ms": ("Stabilisation initiale", "ms"),
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
CV_TREND_MEDIAN_SPAN_MV = 21.0
CV_TREND_GAUSSIAN_SPAN_MV = 45.0


def _rtia_label_for_ohm(ohm):
    if ohm <= 0:
        return RTIA_AUTO_LABEL
    return _rtia_label(min(RTIA_VALUES, key=lambda v: abs(v - ohm)))


def _safe_name(name, fallback):
    """Nettoie un nom pour en faire un composant de chemin valide sous Windows."""
    name = re.sub(r'[<>:"/\\|?*\x00-\x1f]+', "_", (name or "").strip())
    name = name.strip(" .")            # pas d'espace/point en fin (interdits Windows)
    return name or fallback


def _median(values):
    ordered = sorted(float(value) for value in values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) / 2.0


def _window_for_voltage_span(points, start, end, target_mv,
                             minimum=5, maximum=17):
    """Retourne une fenêtre impaire liée à la résolution réelle en tension."""
    deltas = [
        abs(float(points[pos][1]) - float(points[pos - 1][1]))
        for pos in range(start + 1, end)
        if float(points[pos][1]) != float(points[pos - 1][1])
    ]
    step_mv = _median(deltas) if deltas else target_mv
    size = int(round(target_mv / max(step_mv, 1e-9)))
    size = max(minimum, min(maximum, size))
    if size % 2 == 0:
        size += 1 if size < maximum else -1
    branch_size = end - start
    if size > branch_size:
        size = branch_size if branch_size % 2 else branch_size - 1
    return max(1, size)


def _robust_filter_cv_points(points, steps_per_cycle):
    """Extrait la tendance moyenne d'une CV, sans dépendance SciPy.

    Une médiane glissante assez large retire les impulsions, y compris les
    salves de plusieurs échantillons. Une moyenne gaussienne centrée produit
    ensuite une tendance continue sans les ondulations rapides. Les fenêtres
    sont exprimées en mV et s'adaptent donc au nombre de points demandé.
    Les cycles et les branches aller/retour ne sont jamais mélangés.
    """
    result = [tuple(point) for point in points]
    if len(result) < 5:
        return result
    try:
        steps = int(steps_per_cycle)
    except (TypeError, ValueError):
        return result
    if steps < 2:
        return result

    raw = [float(point[2]) for point in points]
    median_trend = list(raw)
    smoothed = list(raw)

    for cycle_start in range(0, len(result), steps):
        cycle_end = min(cycle_start + steps, len(result))
        turn = min(cycle_start + steps // 2, cycle_end)
        for branch_start, branch_end in ((cycle_start, turn), (turn, cycle_end)):
            branch_size = branch_end - branch_start
            if branch_size < 3:
                continue

            # Étape 1 — médiane glissante robuste. Contrairement à un Hampel,
            # elle fournit directement le niveau central local et ne laisse
            # donc pas passer de pic quand le bruit local est déjà élevé.
            median_size = _window_for_voltage_span(
                points, branch_start, branch_end, CV_TREND_MEDIAN_SPAN_MV,
                minimum=7, maximum=31,
            )
            median_radius = median_size // 2
            for pos in range(branch_start, branch_end):
                lo = max(branch_start, pos - median_radius)
                hi = min(branch_end, pos + median_radius + 1)
                median_trend[pos] = _median(raw[lo:hi])

            # Étape 2 — moyenne gaussienne zéro-phase, plus forte que l'ancien
            # Savitzky-Golay. Aux bords, les poids disponibles sont simplement
            # renormalisés afin de ne pas décaler la courbe.
            smooth_size = _window_for_voltage_span(
                points, branch_start, branch_end, CV_TREND_GAUSSIAN_SPAN_MV,
                minimum=15, maximum=61,
            )
            smooth_radius = smooth_size // 2
            sigma = max(smooth_radius / 2.0, 1.0)
            for pos in range(branch_start, branch_end):
                lo = max(branch_start, pos - smooth_radius)
                hi = min(branch_end, pos + smooth_radius + 1)
                weighted_sum = 0.0
                weight_sum = 0.0
                for sample in range(lo, hi):
                    offset = sample - pos
                    weight = math.exp(-0.5 * (offset / sigma) ** 2)
                    weighted_sum += weight * median_trend[sample]
                    weight_sum += weight
                smoothed[pos] = weighted_sum / weight_sum

    for pos, (index, xvalue, _) in enumerate(result):
        result[pos] = (index, xvalue, smoothed[pos])
    return result


# ─────────────────────────────────────────────────────────────────────────────
# Application
# ─────────────────────────────────────────────────────────────────────────────
class CVApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ParisBioTek — Analyse électrochimique (SMP)")
        self.geometry("1180x780")
        self.minsize(900, 560)

        self.smp = None
        self.q = queue.Queue()
        self.busy = False               # une opération série est en cours (thread)
        self.points = []                # (index, volt_mV, current_nA)
        self.last_result = ""
        self.last_params = None         # params envoyés au firmware (dernier run)
        self.last_status = {}           # STATUS renvoyé après le dernier run
        self.connected_status = {}      # STATUS de sonde associé au transport ouvert
        self.last_run_dt = None         # horodatage de la mesure
        self.last_rtia_label = ""       # gamme RTIA demandée (libellé)
        self.last_meta = None           # méta d'affichage/CSV du dernier run
        self.last_error_context = None
        self.last_session_name = ""
        self.last_test_name = ""
        self.last_notes = ""
        self._active_transport = None
        self._stop_requested = threading.Event()
        log_init_error = None
        journal_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "journaux")
        try:
            os.makedirs(journal_dir, exist_ok=True)
            self.log_file_path = os.path.join(
                journal_dir, f"ParisBioTek_{datetime.now():%Y%m%d}.log"
            )
        except OSError as exc:
            self.log_file_path = None
            log_init_error = f"{type(exc).__name__}: {exc}"

        self._build_ui()
        self._refresh_ports()
        self._log(
            "Application démarrée ; journal persistant=" +
            (self.log_file_path or f"indisponible ({log_init_error})")
        )
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
        self.btn_stop = ttk.Button(actions, text="■  Arrêter la mesure",
                                   command=self._request_stop, state="disabled")
        self.btn_stop.pack(fill="x", pady=2)
        self.btn_save = ttk.Button(actions, text="💾  Enregistrer CSV…",
                                   command=self._save_csv, state="disabled")
        self.btn_save.pack(fill="x", pady=2)
        self.autosave_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(actions, text="Enregistrement auto après mesure",
                        variable=self.autosave_var).pack(anchor="w", pady=(4, 0))
        self.filter_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            actions,
            text="Tendance moyenne CV (sans pics)",
            variable=self.filter_var,
            command=self._redraw,
        ).pack(anchor="w", pady=(2, 0))

        logf = ttk.LabelFrame(left, text="Journal", padding=4)
        logf.pack(fill="both", expand=True)
        log_body = ttk.Frame(logf)
        log_body.pack(fill="both", expand=True)
        self.log = tk.Text(log_body, width=42, height=10, state="disabled", wrap="word",
                           font=("Consolas", 8))
        log_scroll = ttk.Scrollbar(log_body, orient="vertical", command=self.log.yview)
        self.log.configure(yscrollcommand=log_scroll.set)
        self.log.pack(side="left", fill="both", expand=True)
        log_scroll.pack(side="right", fill="y")
        log_actions = ttk.Frame(logf)
        log_actions.pack(fill="x", pady=(4, 0))
        ttk.Button(log_actions, text="Copier", command=self._copy_journal).pack(
            side="left", fill="x", expand=True, padx=(0, 2)
        )
        ttk.Button(log_actions, text="Enregistrer…", command=self._save_journal).pack(
            side="left", fill="x", expand=True, padx=(2, 0)
        )

        right = ttk.Frame(root)
        right.grid(row=0, column=1, sticky="nsew")
        right.rowconfigure(0, weight=1)
        right.columnconfigure(0, weight=1)
        self.fig = Figure(figsize=(6, 5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel("Tension appliquée (mV)")
        self.ax.set_ylabel("Courant (µA)")
        self.ax.set_title("Cyclic voltammetry")
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
        self._add_param_row(f, e, "cycles", "Nombre de cycles", DEFAULTS["cycles"])
        self._add_param_row(f, e, "scan_rate", "Vitesse (mV/s)", self._rate_from_defaults())
        self._add_param_row(f, e, "sample_delay_ms", "Délai échantillon (ms)", DEFAULTS["sample_delay_ms"])
        self._add_param_row(f, e, "settle_ms", "Stabilisation initiale (ms)", DEFAULTS["settle_ms"])
        self.entries[METHOD_CV], self.method_frames[METHOD_CV] = e, f

    def _build_ca_params(self):
        f, e = ttk.Frame(self.paramf), {}
        self._add_param_row(f, e, "pot_mv", "Potentiel E = Vbias - Vzero (mV)", CA_DEFAULTS["pot_mv"])
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
                value = float(e[key].get())
            except (TypeError, ValueError):
                raise ValueError("Champs numériques invalides.")
            if not math.isfinite(value):
                raise ValueError(f"Le champ '{key}' doit être un nombre fini.")
            return value

        def inum(key, label):
            value = fnum(key)
            if not value.is_integer():
                raise ValueError(f"{label} doit être un entier.")
            return int(value)

        rtia = RTIA_LABEL_TO_OHM.get(self.rtia_var.get(), 0)

        if mid == METHOD_CV:
            start, peak, vzero = fnum("start_mv"), fnum("peak_mv"), fnum("vzero_mv")
            steps = inum("steps", "Le nombre de pas")
            cycles = inum("cycles", "Le nombre de cycles")
            rate, delay = fnum("scan_rate"), fnum("sample_delay_ms")
            settle_ms = inum("settle_ms", "La stabilisation initiale")
            if steps < 2 or steps % 2 != 0:
                raise ValueError("Le nombre de pas doit être pair et ≥ 2.")
            if cycles < 1:
                raise ValueError("Le nombre de cycles doit être ≥ 1.")
            expected_points = steps * cycles
            if expected_points > CV_MAX_POINTS:
                raise ValueError(
                    f"Nombre total de points trop grand : {expected_points} "
                    f"(maximum {CV_MAX_POINTS} = pas × cycles)."
                )
            if rate <= 0:
                raise ValueError("La vitesse doit être > 0.")
            if delay < 0.1 or delay > 1000.0:
                raise ValueError("Le délai d'échantillonnage doit être entre 0,1 et 1000 ms.")
            if settle_ms < 0 or settle_ms > 60000:
                raise ValueError("La stabilisation initiale doit être entre 0 et 60 000 ms.")
            if peak == start:
                raise ValueError("Départ et sommet doivent différer.")
            start_i, peak_i, vzero_i = int(round(start)), int(round(peak)), int(round(vzero))
            if peak_i == start_i:
                raise ValueError("Après arrondi, départ et sommet doivent différer d'au moins 1 mV.")
            validate_potential_window(vzero_i, start_i, peak_i)
            cycle_dur_ms = int(round(2.0 * abs(peak - start) / rate * 1000.0))
            if cycle_dur_ms < 200 or cycle_dur_ms > 600000:
                raise ValueError("La durée calculée d'un cycle doit être entre 200 et 600 000 ms.")
            if cycle_dur_ms / steps <= delay:
                raise ValueError(
                    "La période par point doit dépasser le délai d'échantillonnage "
                    f"({cycle_dur_ms / steps:.3f} ms ≤ {delay:.3f} ms)."
                )
            total_dur_ms = cycle_dur_ms * cycles + settle_ms
            if total_dur_ms > 0xFFFFFFFF:
                raise ValueError("La durée totale dépasse la capacité du firmware.")
            payload = {"method": METHOD_CV, "start_mv": start_i,
                       "peak_mv": peak_i, "vzero_mv": vzero_i,
                       "steps": steps, "cycles": cycles, "dur_ms": cycle_dur_ms,
                       "delay_us": int(round(delay * 1000.0)),
                       "settle_ms": settle_ms, "rtia": rtia}
            meta = {"dur_ms": total_dur_ms, "cycle_dur_ms": cycle_dur_ms,
                    "cycles": cycles, "steps_per_cycle": steps,
                    "expected_points": expected_points,
                    "xlabel": "Tension appliquée (mV)",
                    "ylabel": "Courant (µA)", "xdiv": 1.0, "xcol": "voltage_mV",
                    "title": f"Cyclic voltammetry — {cycles} cycle{'s' if cycles > 1 else ''}"}
            return payload, meta

        if mid == METHOD_CA:
            pot, vzero = fnum("pot_mv"), fnum("vzero_mv")
            dur = inum("duration_ms", "La durée")
            interval = inum("interval_ms", "L'intervalle")
            if dur < 100:
                raise ValueError("La durée doit être ≥ 100 ms.")
            if dur > 600000:
                raise ValueError("La durée doit être ≤ 600 000 ms.")
            if interval < 5 or interval > min(dur, 10000):
                raise ValueError("Intervalle : 5 ms … min(durée, 10 000 ms).")
            expected_points = max(1, dur // interval)
            if expected_points > CV_MAX_POINTS:
                raise ValueError(
                    f"La CA produirait {expected_points} points (maximum {CV_MAX_POINTS})."
                )
            pot_i, vzero_i = int(round(pot)), int(round(vzero))
            validate_potential_window(vzero_i, pot_i)
            payload = {"method": METHOD_CA, "ca_pot_mv": pot_i,
                       "vzero_mv": vzero_i, "ca_dur_ms": dur,
                       "ca_interval_ms": interval, "rtia": rtia or 4000}
            meta = {"dur_ms": dur, "xlabel": "Temps (s)", "ylabel": "Courant (µA)",
                    "expected_points": expected_points,
                    "xdiv": 1000.0, "xcol": "time_ms", "title": "Chronoamperometry"}
            return payload, meta

        # DPV
        start, end, estep = fnum("start_mv"), fnum("end_mv"), fnum("estep_mv")
        epulse, vzero = fnum("epulse_mv"), fnum("vzero_mv")
        tpulse = inum("tpulse_ms", "La durée d'impulsion")
        tperiod = inum("tperiod_ms", "La période")
        start_i, end_i = int(round(start)), int(round(end))
        estep_i, epulse_i, vzero_i = int(round(estep)), int(round(epulse)), int(round(vzero))
        if end_i == start_i:
            raise ValueError("Départ et fin doivent différer.")
        if estep_i <= 0 or estep_i > 200:
            raise ValueError("Après arrondi, la hauteur de marche doit être entre 1 et 200 mV.")
        if epulse_i == 0 or abs(epulse_i) > 500:
            raise ValueError("Après arrondi, l'impulsion doit être non nulle et ≤ 500 mV.")
        if tpulse < 5 or tpulse > 5000:
            raise ValueError("La durée d'impulsion doit être entre 5 et 5000 ms.")
        if tperiod <= tpulse or tperiod > 20000:
            raise ValueError("La période doit dépasser l'impulsion et rester ≤ 20 000 ms.")
        validate_potential_window(vzero_i, start_i, end_i,
                                  start_i + epulse_i, end_i + epulse_i)
        nsteps = int(abs(end_i - start_i) / estep_i) + 1
        if nsteps > CV_MAX_POINTS:
            raise ValueError(f"La DPV produirait {nsteps} points (maximum {CV_MAX_POINTS}).")
        dur_ms = nsteps * tperiod
        payload = {"method": METHOD_DPV, "start_mv": start_i, "end_mv": end_i,
                   "dpv_estep_mv": estep_i, "dpv_epulse_mv": epulse_i,
                   "dpv_tpulse_ms": tpulse, "dpv_tperiod_ms": tperiod,
                   "vzero_mv": vzero_i, "rtia": rtia or 4000}
        meta = {"dur_ms": dur_ms, "xlabel": "Tension (mV)", "ylabel": "ΔCourant (µA)",
                "expected_points": nsteps,
                "xdiv": 1.0, "xcol": "voltage_mV",
                "title": "Differential pulse voltammetry"}
        return payload, meta

    def _update_duration_label(self):
        lbl = getattr(self, "lbl_duration", None)
        if lbl is None:
            return
        try:
            _, meta = self._read_params()
            if self._current_method_id() == METHOD_CV:
                cycles = meta.get("cycles", 1)
                cycle_dur_ms = meta.get("cycle_dur_ms", meta["dur_ms"])
                if cycles > 1:
                    lbl.config(
                        text=(f"→ 1 cycle ≈ {cycle_dur_ms / 1000:.1f} s "
                              f"· total avec stabilisation ≈ "
                              f"{meta['dur_ms'] / 1000:.1f} s")
                    )
                else:
                    lbl.config(
                        text=f"→ durée avec stabilisation ≈ "
                             f"{meta['dur_ms'] / 1000:.1f} s"
                    )
            else:
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
                    _validate_status(cv_status(s))  # contrat STATUS du groupe CV
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
        self.connected_status = {}
        self.btn_run.config(state="disabled")
        self._set_status(f"Connecté à {port}.")
        self._log(f"Connexion ouverte : port={port}, débit=115200 bauds")
        # Sonde de vie : lit STATUS pour confirmer que c'est bien la carte CV.
        transport = self.smp
        threading.Thread(target=self._probe_worker, args=(transport,), daemon=True).start()

    def _probe_worker(self, transport):
        try:
            st = cv_status(transport)
            _validate_status(st)
            self.q.put(("probe_ok", (transport, st)))
        except Exception as e:
            self.q.put(("probe_fail", {
                "_transport": transport,
                "message": str(e),
                "traceback": traceback.format_exc(),
                "transport": transport.diagnostics_snapshot(),
            }))

    def _disconnect(self, force=False):
        if self.busy and not force:
            messagebox.showinfo(
                "Mesure en cours",
                "Attendez la fin de la mesure avant de déconnecter la carte.",
            )
            return
        if self.smp:
            self.smp.close()
            self.smp = None
        self.connected_status = {}
        self._active_transport = None
        self.btn_connect.config(text="Connecter")
        self.btn_run.config(state="disabled")
        self.btn_stop.config(state="disabled", text="■  Arrêter la mesure")
        self._set_status("Déconnecté.")

    # ── Lancer une mesure ─────────────────────────────────────────────────────
    def _run(self):
        if self.busy or not self.smp:
            return
        blocker = _firmware_measurement_blocker(self.connected_status)
        if blocker:
            self.btn_run.config(state="disabled")
            self._set_status("Firmware à mettre à jour avant toute mesure.")
            self._log("⛔ " + blocker)
            messagebox.showerror("Firmware incompatible", blocker)
            return
        try:
            params, meta = self._read_params()
        except ValueError as e:
            messagebox.showwarning("Paramètres", str(e))
            return
        stop_supported = _firmware_supports_stop(self.connected_status)
        if params.get("method") == METHOD_CV and params.get("settle_ms", 0) > 0 \
                and not stop_supported:
            message = (
                "La stabilisation initiale n'est confirmée qu'avec le firmware "
                "20260825-directinit-v8 (diag_ver=6). Flashez ce firmware ou réglez "
                "Stabilisation initiale à 0 ms pour mesurer avec l'image actuelle."
            )
            self._set_status("Firmware v6 requis pour la stabilisation initiale.")
            self._log("⛔ " + message)
            messagebox.showerror("Firmware incompatible", message)
            return
        self.busy = True
        self._stop_requested.clear()
        self.points = []
        self.last_result = ""
        self.last_params = params
        self.last_meta = meta
        self.last_rtia_label = self.rtia_var.get()
        self.last_run_dt = datetime.now()
        # Figer l'identité opérateur avec les paramètres : une modification des
        # champs pendant l'acquisition ne doit pas déplacer les résultats.
        self.last_session_name = self.session_var.get()
        self.last_test_name = self.test_var.get()
        self.last_notes = self.notes.get("1.0", "end").strip()
        self.btn_run.config(state="disabled", text="⏳  Mesure en cours…")
        self.btn_stop.config(
            state="normal" if stop_supported else "disabled",
            text="■  Arrêter la mesure",
        )
        self.btn_save.config(state="disabled")
        self.btn_connect.config(state="disabled")
        # `STATUS.rtia` indique le RTIA matériel courant. Avant la première
        # sonde, ce peut encore être l'ancienne gamme (souvent 4 kΩ) alors que
        # l'utilisateur a demandé Auto ; ne pas l'afficher comme une sélection
        # finale et trompeuse pendant le lancement.
        if params.get("method") == METHOD_CV and _as_int(params.get("rtia")) == 0:
            self.lbl_rtia.config(text="RTIA : auto-range (sondes avant la CV)")
        self._set_status(f"Démarrage — {meta['title']}…")
        transport = self.smp
        self._active_transport = transport
        self._log(
            f"NOUVELLE MESURE : {meta['title']} ; port={transport.port} ; "
            f"durée prévue={meta.get('dur_ms', '—')} ms ; "
            f"points attendus={meta.get('expected_points', '—')} ; paramètres={params}"
        )
        if not stop_supported:
            self._log(
                "⚠ Bouton Stop indisponible : le firmware connecté annonce "
                f"diag_ver={self.connected_status.get('diag_ver', '—')} ; diagnostic v6 requis."
            )
        threading.Thread(
            target=self._run_worker,
            args=(transport, params, meta, self._stop_requested), daemon=True
        ).start()

    def _request_stop(self):
        """Signale le worker sans effectuer d'E/S série dans le thread Tk."""
        if not self.busy or self._active_transport is None:
            return
        if not _firmware_supports_stop(self.connected_status):
            message = (
                "Arrêt propre indisponible avec ce firmware ; "
                "20260825-directinit-v8 (diag_ver=6) est requis."
            )
            self._log("⚠ " + message)
            messagebox.showinfo("Arrêt indisponible", message)
            return
        if self._stop_requested.is_set():
            return
        self._stop_requested.set()
        self.btn_stop.config(state="disabled", text="⏳  Arrêt demandé…")
        self._set_status("Arrêt demandé ; récupération des points en cours…")
        self._log(
            "ARRÊT UTILISATEUR demandé ; le worker va envoyer STOP puis "
            "récupérer tous les points confirmés par STATUS."
        )

    def _run_worker(self, transport, params, meta, stop_event=None):
        if stop_event is None:
            stop_event = threading.Event()
        pts = []
        off = 0
        n = 0
        last_st = {}
        start_reply = {}
        timeline = []
        run_id = None
        release_required = False
        released = False
        success_payload = None
        stopped_payload = None
        error_context = None
        stop_sent = False
        stop_ack = None
        latest_data_n = 0
        release_diagnostics = {
            "required": False, "attempts": 0, "released": False, "errors": []
        }
        previous_n = None
        started_wall = datetime.now().astimezone().isoformat(timespec="milliseconds")
        t0 = time.monotonic()

        def download_until(target):
            nonlocal off, latest_data_n
            while off < target:
                reply = cv_data(transport, off, run_id)
                reply_n, pairs = _validate_data_reply(
                    reply, off, run_id=run_id, max_points=CV_MAX_POINTS
                )
                latest_data_n = max(latest_data_n, reply_n)
                if reply_n < target:
                    raise SmpProtocolError(
                        f"DATA ne contient plus tous les points annoncés "
                        f"(n DATA={reply_n}, cible STATUS={target})"
                    )
                if not pairs:
                    raise SmpProtocolError(
                        f"lot DATA vide à l'offset {off}, alors que {target} point(s) sont annoncés"
                    )
                base_off = off
                for index, (xval, current) in enumerate(pairs):
                    pts.append((base_off + index, xval, current))
                off += len(pairs)
                expected = meta.get("expected_points")
                if expected is not None and off > int(expected):
                    raise SmpProtocolError(
                        f"trop de données reçues ({off} > {expected} points attendus)"
                    )
                self.q.put(("download", (off, max(target, reply_n))))

        def release_buffer():
            nonlocal released, n
            if released or not release_required or run_id is None:
                return
            # DATA peut observer un point arrivé entre deux STATUS (cas réel :
            # STATUS n=3198 puis DATA n=3199). Acquitter le compteur le plus
            # récent au lieu de réutiliser aveuglément le vieux STATUS.
            n = max(n, latest_data_n, off)
            release_diagnostics["attempts"] += 1
            try:
                reply = cv_release(transport, run_id, n)
                _validate_release_reply(reply, run_id, n)
            except Exception as exc:
                release_diagnostics["errors"].append(
                    f"{type(exc).__name__}: {exc}"
                )
                raise
            released = True
            release_diagnostics["released"] = True
            release_diagnostics["ack"] = reply
            self.q.put(("log", f"Buffer firmware libéré (run_id={run_id}, n={n})."))

        def read_status_resilient(context, attempts=3):
            """Relance uniquement un STATUS perdu, sans masquer une réponse invalide."""
            for attempt in range(1, attempts + 1):
                try:
                    return cv_status(transport)
                except SmpProtocolError:
                    raise
                except SmpError as exc:
                    # Les journaux terrain montrent une ligne base64 perdue au
                    # passage run=true -> complete. La mesure est encore saine;
                    # une nouvelle requête STATUS est idempotente et sûre.
                    if ("pas de réponse SMP" not in str(exc) or
                            "timeout" not in str(exc) or attempt >= attempts):
                        raise
                    self.q.put((
                        "log",
                        f"⚠ STATUS timeout transitoire ({context}), "
                        f"nouvelle tentative {attempt + 1}/{attempts}."
                    ))
                    time.sleep(0.15)
            raise AssertionError("boucle STATUS inaccessible")

        def refresh_release_state():
            """Resynchronise STATUS/DATA avant un RELEASE de récupération."""
            nonlocal n, previous_n, last_st
            refreshed = read_status_resilient("resynchronisation RELEASE")
            _validate_status(
                refreshed, previous_n=previous_n, previous_run_id=run_id,
                expected_points=meta.get("expected_points"),
            )
            last_st = refreshed
            n = refreshed["n"]
            previous_n = n
            timeline.append(_status_timeline_entry(
                refreshed, (time.monotonic() - t0) * 1000
            ))
            self.q.put(("log", _status_summary(
                refreshed, "STATUS resynchronisation RELEASE"
            )))
            if n > off:
                download_until(n)
                self.q.put(("live_points", pts.copy()))
            return refreshed

        def send_stop_if_requested():
            """Envoie STOP une seule fois, depuis l'unique worker propriétaire du port."""
            nonlocal stop_sent, stop_ack
            if not stop_event.is_set() or stop_sent or run_id is None:
                return
            stop_sent = True
            self.q.put(("log", f"Envoi STOP pour run_id={run_id}…"))
            try:
                stop_ack = _validate_stop_reply(cv_stop(transport, run_id), run_id)
            except SmpProtocolError as exc:
                raise SmpProtocolError(
                    "Le firmware n'implémente pas l'ACK STOP v6 attendu "
                    f"(20260825-directinit-v8 requis) : {exc}"
                ) from exc
            except SmpError as exc:
                if "erreur SMP rc=" in str(exc):
                    raise SmpProtocolError(
                        "La commande STOP a été rejetée ; firmware "
                        f"20260825-directinit-v8 requis : {exc}"
                    ) from exc
                raise
            self.q.put(("log",
                f"STOP ACK : run_id={stop_ack['run_id']}, "
                f"stop_requested={stop_ack['stop_requested']}, n={stop_ack['n']}."
            ))

        try:
            start_reply = _validate_start_reply(cv_start(transport, params))
            if "run_id" in start_reply:
                run_id = _required_int(start_reply, "run_id", minimum=0)
            if "release_required" in start_reply:
                release_required = start_reply["release_required"]
                release_diagnostics["required"] = release_required
            if _as_int(start_reply.get("diag_ver"), -1) >= 6:
                accepted_settle = _required_int(
                    start_reply, "settle_ms", minimum=0, maximum=60000
                )
                if accepted_settle != int(params.get("settle_ms", 0)):
                    raise SmpProtocolError(
                        "Le firmware a modifié la stabilisation initiale "
                        f"({accepted_settle} ms au lieu de {params.get('settle_ms', 0)} ms)."
                    )
            self.q.put(("log", _start_ack_summary(start_reply, params, meta)))
            # Les firmwares récents confirment la configuration retenue. Les
            # anciens ignorent une clé SMP inconnue : l'absence de ces champs
            # est tolérée, puis détectée par le nombre final de points.
            if params.get("method") == METHOD_CV and "cycles" in start_reply:
                accepted_cycles = _required_int(start_reply, "cycles", minimum=1)
                accepted_steps = _required_int(start_reply, "steps", minimum=2)
                accepted_total = _required_int(start_reply, "total_points", minimum=1)
                if accepted_cycles != int(params.get("cycles", 1)):
                    self.q.put(("log",
                        f"⚠ Le firmware a retenu {accepted_cycles} cycle(s), "
                        f"au lieu de {params.get('cycles', 1)}."
                    ))
                if accepted_steps != int(params["steps"]):
                    self.q.put(("log", "⚠ Le firmware a modifié le nombre de pas demandé."))
                if accepted_total != int(meta["expected_points"]):
                    self.q.put(("log", "⚠ Le firmware a retenu un nombre de points inattendu."))
            self.q.put(("log", "START accepté ; attente de l'initialisation AD5940."))

            overhead_s = 45.0 if (params.get("method") == METHOD_CV and
                                  _as_int(params.get("rtia")) == 0) else 25.0
            max_s = meta["dur_ms"] / 1000.0 + overhead_s
            last_log_at = 0.0
            last_stage = None
            expected = meta.get("expected_points")

            while time.monotonic() - t0 < max_s:
                send_stop_if_requested()
                time.sleep(0.4)
                send_stop_if_requested()
                st = read_status_resilient("acquisition")
                # Conserver aussi une réponse mal formée dans le rapport : la
                # validation peut précisément être la cause de l'exception.
                last_st = st if isinstance(st, dict) else {"raw_status": repr(st)}
                candidate_run_id = st.get("run_id") if isinstance(st, dict) else None
                # Le premier STATUS fixe l'identité de la mesure. Les suivants
                # doivent rester sur la même acquisition.
                if run_id is None and candidate_run_id is not None:
                    run_id = candidate_run_id
                _validate_status(
                    st, previous_n=previous_n, previous_run_id=run_id,
                    expected_points=expected,
                )
                n = st["n"]
                self.q.put(("status_snapshot", dict(st)))
                previous_n = n
                elapsed_s = time.monotonic() - t0
                timeline.append(_status_timeline_entry(st, elapsed_s * 1000))

                # Toujours récupérer les points disponibles avant de lever une
                # erreur firmware : ils seront tracés et sauvegardés en partiel.
                if n > off:
                    try:
                        download_until(n)
                        self.q.put(("live_points", pts.copy()))
                    except Exception as download_exc:
                        if st["err"] == 0:
                            raise
                        # La panne firmware est la cause primaire. Une erreur
                        # DATA secondaire ne doit pas la remplacer dans le
                        # dialogue ni dans le rapport automatique.
                        raise SmpError(
                            _measurement_error_message(st, params, meta, start_reply)
                            + "\nLa récupération des points partiels a aussi échoué : "
                            + f"{type(download_exc).__name__}: {download_exc}"
                        ) from download_exc

                stage = st.get("stage")
                now = time.monotonic()
                if stage != last_stage or now - last_log_at >= 2.0:
                    observed_rate = n / elapsed_s if elapsed_s > 0 else 0.0
                    expected_rate = ((expected or 0) / (meta["dur_ms"] / 1000.0)
                                     if meta.get("dur_ms") else 0.0)
                    self.q.put(("log",
                        _status_summary(st) +
                        f"\n  Cadence client : {observed_rate:.2f} point(s)/s "
                        f"(nominale {expected_rate:.2f}), téléchargés={off}."
                    ))
                    last_log_at, last_stage = now, stage

                self.q.put(("progress", n))
                if st["err"] != 0:
                    if st["run"]:
                        # L'erreur est déjà informative, mais laisser le worker
                        # firmware terminer son abort afin que RELEASE soit
                        # accepté avec le nombre final de points.
                        continue
                    try:
                        release_buffer()
                    except Exception as release_exc:
                        self.q.put(("log",
                            "⚠ RELEASE après erreur reporté : "
                            f"{type(release_exc).__name__}: {release_exc}"
                        ))
                    raise SmpError(_measurement_error_message(st, params, meta, start_reply))
                if not st["run"]:
                    if n == 0 and not (
                            st.get("cancelled") is True and stop_ack is not None and
                            stop_ack.get("stop_requested") is True):
                        raise SmpError(
                            "La mesure s'est arrêtée sans erreur déclarée et sans aucun point."
                        )
                    break
            else:
                self.q.put(("log", _status_summary(last_st, "STATUS timeout client")))
                timeout_suffix = (
                    f"\nLe firmware n'a pas terminé son arrêt avant le délai client "
                    f"({time.monotonic() - t0:.1f} s)."
                )
                if (isinstance(last_st, dict) and
                        isinstance(last_st.get("err"), int) and last_st["err"] != 0):
                    raise SmpError(
                        _measurement_error_message(last_st, params, meta, start_reply)
                        + timeout_suffix
                    )
                raise SmpError(
                    f"Délai client dépassé après {time.monotonic() - t0:.1f} s "
                    f"({n} point(s), phase={last_st.get('stage', '—')})."
                )

            # Un dernier STATUS ferme la fenêtre entre le dernier poll et la fin.
            st = read_status_resilient("confirmation finale")
            last_st = st if isinstance(st, dict) else {"raw_status": repr(st)}
            _validate_status(
                st, previous_n=previous_n, previous_run_id=run_id,
                expected_points=expected,
            )
            n = st["n"]
            if n > off:
                try:
                    download_until(n)
                except Exception as download_exc:
                    if st["err"] == 0:
                        raise
                    raise SmpError(
                        _measurement_error_message(st, params, meta, start_reply)
                        + "\nLa récupération finale des points partiels a aussi échoué : "
                        + f"{type(download_exc).__name__}: {download_exc}"
                    ) from download_exc
            self.q.put(("live_points", pts.copy()))
            if st["err"] != 0:
                self.q.put(("log", _status_summary(st, "STATUS erreur finale")))
                try:
                    release_buffer()
                except Exception as release_exc:
                    self.q.put(("log",
                        "⚠ RELEASE après erreur reporté : "
                        f"{type(release_exc).__name__}: {release_exc}"
                    ))
                raise SmpError(_measurement_error_message(st, params, meta, start_reply))
            if st["run"]:
                raise SmpProtocolError("le STATUS final indique encore run=true")

            cancelled = st.get("cancelled") is True
            if cancelled:
                if stop_ack is None or stop_ack.get("stop_requested") is not True:
                    raise SmpProtocolError(
                        "Le firmware annonce cancelled=true sans ACK STOP utilisateur."
                    )
                if st.get("stage") != "cancelled":
                    raise SmpProtocolError(
                        "STATUS d'arrêt utilisateur incohérent : "
                        f"stage={st.get('stage', '—')!r}, attendu 'cancelled'."
                    )
                try:
                    release_buffer()
                except Exception as release_exc:
                    self.q.put(("log",
                        "⚠ Mesure interrompue récupérée, mais RELEASE a échoué : "
                        f"{type(release_exc).__name__}: {release_exc}"
                    ))
                stopped_payload = (pts, st, stop_ack)
            elif expected is not None and len(pts) != int(expected):
                if (params.get("method") == METHOD_CV and
                        int(params.get("cycles", 1)) > 1 and
                        len(pts) == int(params["steps"])):
                    raise SmpError(
                        f"Le firmware n'a renvoyé qu'un seul cycle "
                        f"({len(pts)} points au lieu de {expected})."
                    )
                raise SmpError(
                    f"Mesure incomplète : {len(pts)} point(s) reçu(s), {expected} attendu(s)."
                )
            if not cancelled:
                try:
                    release_buffer()
                except Exception as release_exc:
                    self.q.put(("log",
                        "⚠ La mesure est complète, mais RELEASE a échoué : "
                        f"{type(release_exc).__name__}: {release_exc}"
                    ))
                success_payload = (pts, st)
        except Exception as exc:
            failure_hint = _known_transport_failure_hint(
                exc, last_st, meta.get("expected_points")
            )
            error_context = {
                "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                "started_at": started_wall,
                "client_elapsed_ms": round((time.monotonic() - t0) * 1000),
                "exception_type": type(exc).__name__,
                "message": str(exc) + failure_hint,
                "traceback": traceback.format_exc(),
                "params": params,
                "meta": meta,
                "start_reply": start_reply,
                "last_status": last_st,
                "timeline": timeline,
                "points": pts,
                "points_received": len(pts),
                "transport": transport.diagnostics_snapshot(),
            }
            if isinstance(exc, SmpError) and exc.context:
                error_context["exception_context"] = exc.context
        finally:
            # Le firmware protège le buffer SMP contre une mesure BLE jusqu'à
            # cette confirmation. En cas d'erreur détectée alors que le worker
            # firmware termine encore son abort, quelques tentatives courtes
            # évitent de laisser le bail actif jusqu'à son expiration.
            if release_required and run_id is not None and not released:
                release_exc = None
                attempts = 4 if isinstance(last_st, dict) and last_st.get("run") else 1
                for attempt in range(attempts):
                    try:
                        if (attempt > 0 or
                                (isinstance(last_st, dict) and last_st.get("run")) or
                                max(latest_data_n, off) != n):
                            refreshed = refresh_release_state()
                            if refreshed.get("run"):
                                raise SmpProtocolError(
                                    "RELEASE reporté : le firmware indique encore run=true"
                                )
                        release_buffer()
                        release_exc = None
                        break
                    except Exception as exc:
                        release_exc = exc
                        if attempt + 1 < attempts:
                            time.sleep(0.25)
                if release_exc is not None:
                    self.q.put(("log",
                        "⚠ Impossible de confirmer la libération du buffer "
                        f"run_id={run_id} : {type(release_exc).__name__}: {release_exc}. "
                        "Le firmware le libérera automatiquement après 60 s."
                    ))
            release_diagnostics["required"] = release_required
            release_diagnostics["released"] = released
            release_diagnostics["run_id"] = run_id
            release_diagnostics["n"] = n

            # Publier l'issue à la GUI seulement après la dernière tentative
            # RELEASE : elle ne peut ainsi ni relancer ni fermer le transport
            # pendant que l'ancien worker l'utilise encore.
            if error_context is not None:
                error_context["release"] = dict(release_diagnostics)
                error_context["last_status_after_release"] = last_st
                try:
                    error_context["transport"] = transport.diagnostics_snapshot()
                except Exception as diag_exc:
                    error_context["transport_after_release_error"] = (
                        f"{type(diag_exc).__name__}: {diag_exc}"
                    )
                self.q.put(("run_err", error_context))
            elif success_payload is not None:
                self.q.put(("run_done", success_payload))
            elif stopped_payload is not None:
                self.q.put(("run_stopped", stopped_payload))

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
                    transport, probe_status = payload
                    # Une sonde peut finir après une déconnexion/reconnexion.
                    # Ne jamais appliquer son résultat à un autre port ouvert.
                    if self.smp is not transport:
                        continue
                    self.connected_status = dict(probe_status)
                    self._log(_status_summary(probe_status, "SONDE STATUS"))
                    blocker = _firmware_measurement_blocker(probe_status)
                    if blocker:
                        self._set_status("Firmware incompatible : mise à jour requise.")
                        self.btn_run.config(state="disabled")
                        self._log("⛔ " + blocker)
                        messagebox.showerror("Firmware incompatible", blocker)
                    else:
                        self._set_status("Carte CV connectée (SMP OK).")
                        self.btn_run.config(state="normal")
                    missing = _cv_cycle_protocol_missing(probe_status)
                    if missing:
                        self._log(
                            "⚠ Firmware sans signature de protocole CV multi-cycle "
                            f"(champs absents : {', '.join(missing)}). "
                            "Une mise à jour peut être nécessaire."
                        )
                    else:
                        self._log(
                            "Firmware CV multi-cycle confirmé : "
                            f"{probe_status.get('steps')} pas/cycle × "
                            f"{probe_status.get('cycles')} cycle(s), "
                            f"{probe_status.get('total_points')} point(s)."
                        )
                    self._apply_rtia(probe_status.get("rtia", 0))
                elif kind == "probe_fail":
                    if self.smp is not payload.get("_transport"):
                        continue
                    self._set_status("Connecté, mais pas de réponse SMP CV — mauvais port ?")
                    self.btn_run.config(state="disabled")
                    self._log(
                        "⚠ Sonde SMP échouée : " + payload.get("message", "erreur inconnue") +
                        f"\n  Transport : {payload.get('transport', {})}" +
                        f"\n  Traceback :\n{payload.get('traceback', '—')}"
                    )
                elif kind == "progress":
                    expected = (self.last_meta or {}).get("expected_points")
                    if expected:
                        self._set_status(f"Mesure en cours… {payload}/{expected} points")
                    else:
                        self._set_status(f"Mesure en cours… {payload} points")
                elif kind == "status_snapshot":
                    self.last_status = payload
                elif kind == "download":
                    off, n = payload
                    self._set_status(f"Téléchargement… {off}/{n}")
                elif kind == "live_points":
                    self.points = payload
                    self._redraw()
                elif kind == "run_done":
                    self._finish_run(*payload)
                elif kind == "run_stopped":
                    self._finish_stopped(*payload)
                elif kind == "run_err":
                    self.busy = False
                    self._active_transport = None
                    self.btn_run.config(state="normal", text="▶  Lancer la mesure")
                    self.btn_stop.config(state="disabled", text="■  Arrêter la mesure")
                    self.btn_connect.config(state="normal")
                    self._set_status("Échec mesure.")
                    self.last_error_context = payload
                    self.points = list(payload.get("points") or [])
                    self.last_status = payload.get("last_status") or {}
                    self.last_result = (
                        f"ERREUR={self.last_status.get('err', 'client')},"
                        f"points_partiels={len(self.points)}"
                    )
                    if self.points:
                        self.btn_save.config(state="normal")
                        self._redraw()
                    report_paths = self._save_error_diagnostics(payload)
                    report_text = report_paths.get("report") or "rapport non écrit"
                    self._log(
                        "ÉCHEC MESURE\n"
                        f"  Type : {payload.get('exception_type', '—')}\n"
                        f"  Message : {payload.get('message', '—')}\n"
                        f"  Points partiels conservés : {len(self.points)}\n"
                        + _status_summary(self.last_status, "  Dernier STATUS") + "\n"
                        f"  Transport : {payload.get('transport', {})}\n"
                        f"  Rapport diagnostic : {report_text}"
                    )
                    modal = payload.get("message", "Erreur de mesure")
                    if self.points:
                        modal += f"\n\n{len(self.points)} point(s) partiel(s) ont été conservés."
                    modal += f"\n\nRapport diagnostic :\n{report_text}"
                    messagebox.showerror("Mesure", modal)
        except queue.Empty:
            pass
        except Exception:
            # Une erreur d'affichage ne doit pas tuer définitivement la boucle
            # de messages. Le traceback complet reste dans le journal fichier.
            try:
                self._log("⚠ Erreur interne de la boucle GUI :\n" + traceback.format_exc())
            except Exception:
                pass
        finally:
            self.after(60, self._poll_queue)

    def _apply_rtia(self, ohm):
        try:
            ohm = int(ohm)
            self.lbl_rtia.config(text=f"RTIA actif : {ohm} Ω" if ohm else "RTIA : auto-range")
        except (ValueError, TypeError):
            pass

    def _finish_run(self, pts, st):
        self.busy = False
        self._active_transport = None
        self.points = pts
        self.last_status = st or {}
        self.btn_run.config(state="normal", text="▶  Lancer la mesure")
        self.btn_stop.config(state="disabled", text="■  Arrêter la mesure")
        self.btn_connect.config(state="normal")
        if pts:
            self.btn_save.config(state="normal")
        peak_na = int(st.get("peak_na", 0))
        peak_ua = peak_na / 1000.0
        r = st.get("r_ohm", 0)
        self.last_result = f"peak_uA={peak_ua:.3f},R_ohm={r}"
        self._apply_rtia(st.get("rtia", 0))
        cycles = int((self.last_meta or {}).get("cycles", 1))
        cycle_text = f", {cycles} cycle{'s' if cycles > 1 else ''}" if cycles > 1 else ""
        self._set_status(f"Terminé — {len(pts)} points{cycle_text}. {self.last_result}")
        self._log(
            f"RESULT: {self.last_result}  RTIA={st.get('rtia', 0)} Ω "
            f"(cal={st.get('rtia_cal', '—')} Ω)"
        )
        if (self.last_params or {}).get("method") == METHOD_CV:
            self._log(
                f"Filtre numérique Python : médiane robuste sur "
                f"~{CV_TREND_MEDIAN_SPAN_MV:g} mV + moyenne gaussienne sur "
                f"~{CV_TREND_GAUSSIAN_SPAN_MV:g} mV, par branche ; les courants bruts "
                "restent conservés dans le CSV."
            )
        self._log(_status_summary(st, "STATUS final"))
        if st.get("method") in (METHOD_CA, METHOD_DPV) and "dac_ok" in st:
            self._log(
                "LPDAC: "
                f"E={st.get('pot_mv', '—')} mV, Vzero={st.get('vzero_mv', '—')} mV, "
                f"VbiasCode={st.get('dac_vbias', '—')}, "
                f"VzeroCode={st.get('dac_vzero', '—')}, "
                f"reg=0x{int(st.get('dac_reg', 0)):05X}, "
                f"readback={'OK' if st.get('dac_ok') else 'ECHEC'}"
            )
        self._redraw()
        if self.autosave_var.get() and pts:
            self._save_csv(auto=True)

    def _finish_stopped(self, pts, st, stop_ack):
        """Publie un arrêt opérateur comme résultat partiel, jamais comme erreur."""
        self.busy = False
        self._active_transport = None
        self.points = list(pts)
        self.last_status = st or {}
        self.btn_run.config(state="normal", text="▶  Lancer la mesure")
        self.btn_stop.config(state="disabled", text="■  Arrêter la mesure")
        self.btn_connect.config(state="normal")
        if self.points:
            self.btn_save.config(state="normal")
        self.last_result = (
            "statut=interrompue par utilisateur,"
            f"points_partiels={len(self.points)}"
        )
        self._apply_rtia(self.last_status.get("rtia", 0))
        self._set_status(
            f"Mesure interrompue par utilisateur — {len(self.points)} point(s) conservé(s)."
        )
        self._log(
            "MESURE INTERROMPUE PAR UTILISATEUR\n"
            f"  Points conservés : {len(self.points)}\n"
            f"  ACK STOP : {stop_ack}\n" +
            _status_summary(self.last_status, "  STATUS final")
        )
        self._redraw()
        if self.autosave_var.get() and self.points:
            self._save_csv(auto=True)

    # ── Tracé ────────────────────────────────────────────────────────────────
    def _redraw(self):
        meta = self.last_meta or {}
        xdiv = meta.get("xdiv", 1.0)
        if not self.points:
            self.curve.set_data([], [])
        else:
            display_points = self.points
            method_id = (self.last_params or {}).get("method", METHOD_CV)
            filter_enabled = bool(
                getattr(self, "filter_var", None) and self.filter_var.get()
            )
            if method_id == METHOD_CV and filter_enabled:
                display_points = _robust_filter_cv_points(
                    self.points,
                    (self.last_params or {}).get("steps", len(self.points)),
                )
            xs = [x / xdiv for (_, x, _) in display_points]   # ms→s en CA
            ys = [c / 1000.0 for (_, _, c) in display_points] # nA → µA
            self.curve.set_data(xs, ys)
            self.ax.relim()
            self.ax.autoscale_view()
        self.ax.set_xlabel(meta.get("xlabel", "Tension appliquée (mV)"))
        self.ax.set_ylabel(meta.get("ylabel", "Courant (µA)"))
        title = meta.get("title", "Cyclic voltammetry")
        if ((self.last_params or {}).get("method", METHOD_CV) == METHOD_CV and
                getattr(self, "filter_var", None) and self.filter_var.get()):
            title += " — tendance moyenne"
        self.ax.set_title(title)
        self.canvas.draw_idle()

    # ── CSV + métadonnées ───────────────────────────────────────────────────────
    def _save_csv(self, auto=False):
        if not self.points:
            if not auto:
                messagebox.showinfo("Enregistrer", "Aucune donnée.")
            return

        session = _safe_name(self.last_session_name, "SansSession")
        test = _safe_name(self.last_test_name, "test")
        dt = self.last_run_dt or datetime.now()
        stamp = dt.strftime("%Y%m%d_%H%M%S_%f")
        interrupted_suffix = "_INTERROMPUE" if self.last_status.get("cancelled") is True else ""
        base = f"{test}_{stamp}{interrupted_suffix}.csv"
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
        method_id = (self.last_params or {}).get("method", 0)
        method_name = self._method_name(method_id)
        is_cv = method_id == METHOD_CV
        cycles = int((self.last_params or {}).get("cycles", 1)) if is_cv else 1
        steps_per_cycle = int((self.last_params or {}).get("steps", 1)) if is_cv else 1
        filtered_points = (
            _robust_filter_cv_points(self.points, steps_per_cycle)
            if is_cv else list(self.points)
        )
        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                header = (f"# session={session} test={test} "
                          f"method={method_name} date={dt:%Y-%m-%d %H:%M:%S}")
                if is_cv:
                    header += (
                        f" cycles={cycles} steps_per_cycle={steps_per_cycle}"
                        f" cycle_duration_ms={meta.get('cycle_dur_ms', '—')}"
                        f" total_duration_ms={meta.get('dur_ms', '—')}"
                    )
                w.writerow([header])
                if self.last_result:
                    w.writerow(["# " + self.last_result])
                if is_cv:
                    w.writerow([
                        f"# filtre_numerique=median_gaussian_trend "
                        f"median_span_mV={CV_TREND_MEDIAN_SPAN_MV:g} "
                        f"gaussian_span_mV={CV_TREND_GAUSSIAN_SPAN_MV:g} "
                        "par_branche=true donnees_brutes_preservees=true"
                    ])
                if is_cv and cycles > 1:
                    w.writerow([
                        "index", "cycle", xcol, "current_nA", "current_uA",
                        "current_filtered_nA", "current_filtered_uA",
                    ])
                    for (idx, x, cur_na), (_, _, filtered_na) in zip(
                            self.points, filtered_points):
                        cycle = idx // steps_per_cycle + 1
                        w.writerow([
                            idx, cycle, f"{x:.1f}", f"{cur_na:.1f}",
                            f"{cur_na / 1000.0:.4f}", f"{filtered_na:.1f}",
                            f"{filtered_na / 1000.0:.4f}",
                        ])
                elif is_cv:
                    w.writerow([
                        "index", xcol, "current_nA", "current_uA",
                        "current_filtered_nA", "current_filtered_uA",
                    ])
                    for (idx, x, cur_na), (_, _, filtered_na) in zip(
                            self.points, filtered_points):
                        w.writerow([
                            idx, f"{x:.1f}", f"{cur_na:.1f}",
                            f"{cur_na / 1000.0:.4f}", f"{filtered_na:.1f}",
                            f"{filtered_na / 1000.0:.4f}",
                        ])
                else:
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
        meta = self.last_meta or {}
        notes = self.last_notes
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
        if method_id == METHOD_CV:
            lines += [
                f"  {'Durée totale':20}: {meta.get('dur_ms', '—')} ms",
                f"  {'Points attendus':20}: {meta.get('expected_points', '—')}",
                f"  {'Filtre numérique':20}: médiane robuste / "
                f"{CV_TREND_MEDIAN_SPAN_MV:g} mV + moyenne gaussienne / "
                f"{CV_TREND_GAUSSIAN_SPAN_MV:g} mV, par branche",
                f"  {'Données brutes':20}: conservées dans current_nA/current_uA",
            ]
        lines.append(f"  {'Gamme RTIA (libellé)':20}: {self.last_rtia_label or '—'}")

        lines += [
            "",
            "Résultat :",
            f"  {'État':20}: " +
            ("interrompue par utilisateur" if st.get("cancelled") is True else "terminée"),
            f"  {'Points collectés':20}: {len(self.points)}",
            f"  {'Pic':20}: {st.get('peak_na', '—')} nA",
            f"  {'RTIA actif':20}: {st.get('rtia', '—')} Ω",
            f"  {'RTIA calibré':20}: {st.get('rtia_cal', '—')} Ω",
        ]
        if method_id == METHOD_CV:
            lines.append(f"  {'Résistance estimée':20}: {st.get('r_ohm', '—')} Ω")
        if method_id in (METHOD_CA, METHOD_DPV):
            lines += [
                f"  {'LPDAC E demandé':20}: {st.get('pot_mv', '—')} mV",
                f"  {'LPDAC Vzero':20}: {st.get('vzero_mv', '—')} mV",
                f"  {'LPDAC Vbias code':20}: {st.get('dac_vbias', '—')}",
                f"  {'LPDAC Vzero code':20}: {st.get('dac_vzero', '—')}",
                f"  {'LPDAC registre':20}: 0x{int(st.get('dac_reg', 0)):05X}",
                f"  {'LPDAC lecture':20}: {'OK' if st.get('dac_ok') else 'échec'}",
            ]
        lines += ["", "Notes :", notes if notes else "(aucune)", ""]

        with open(txt_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))

    def _save_error_diagnostics(self, context):
        """Écrit automatiquement rapport JSON/TXT et CSV partiel après un échec."""
        session = _safe_name(self.last_session_name, "SansSession")
        test = _safe_name(self.last_test_name, "test")
        dt = self.last_run_dt or datetime.now()
        stamp = dt.strftime("%Y%m%d_%H%M%S_%f")
        folder = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "mesures", session, test
        )
        paths = {}
        try:
            os.makedirs(folder, exist_ok=True)
            stem = os.path.join(folder, f"{test}_{stamp}_ERREUR")
            json_path = stem + "_diagnostic.json"
            report_path = stem + ".txt"
            partial_path = stem + "_partiel.csv"

            serializable = dict(context)
            serializable["session"] = session
            serializable["test"] = test
            serializable["notes"] = self.last_notes
            serializable["gui_journal_file"] = self.log_file_path
            with open(json_path, "w", encoding="utf-8") as stream:
                json.dump(serializable, stream, ensure_ascii=False, indent=2, default=str)

            status = context.get("last_status") or {}
            lines = [
                "ParisBioTek — rapport automatique d'erreur",
                "=" * 52,
                f"Date              : {context.get('timestamp', '—')}",
                f"Début             : {context.get('started_at', '—')}",
                f"Session / test    : {session} / {test}",
                f"Type d'exception  : {context.get('exception_type', '—')}",
                f"Message           : {context.get('message', '—')}",
                f"Temps client      : {context.get('client_elapsed_ms', '—')} ms",
                f"Points conservés  : {context.get('points_received', 0)}",
                "",
                _status_summary(status, "Dernier STATUS firmware"),
                "",
                "Paramètres envoyés :",
                json.dumps(context.get("params", {}), ensure_ascii=False, indent=2),
                "",
                "Réponse START :",
                json.dumps(context.get("start_reply", {}), ensure_ascii=False, indent=2),
                "",
                "Acquittement/libération du buffer :",
                json.dumps(context.get("release", {}), ensure_ascii=False, indent=2),
                "",
                "Diagnostic transport SMP :",
                json.dumps(context.get("transport", {}), ensure_ascii=False, indent=2),
                "",
                "Chronologie STATUS :",
                json.dumps(context.get("timeline", []), ensure_ascii=False, indent=2),
                "",
                "Traceback Python :",
                context.get("traceback", "—"),
                "",
                "Notes opérateur :",
                serializable["notes"] or "(aucune)",
                "",
            ]
            with open(report_path, "w", encoding="utf-8") as stream:
                stream.write("\n".join(lines))

            points = context.get("points") or []
            if points:
                meta = context.get("meta") or {}
                params = context.get("params") or {}
                is_cv = params.get("method") == METHOD_CV
                steps_per_cycle = int(params.get("steps", len(points) or 1))
                filtered_points = (
                    _robust_filter_cv_points(points, steps_per_cycle)
                    if is_cv else list(points)
                )
                with open(partial_path, "w", newline="", encoding="utf-8") as stream:
                    writer = csv.writer(stream)
                    writer.writerow([
                        f"# ERREUR mesure partielle session={session} test={test} "
                        f"date={dt:%Y-%m-%d %H:%M:%S}"
                    ])
                    writer.writerow(["# " + str(context.get("message", "erreur inconnue"))])
                    if is_cv:
                        writer.writerow([
                            f"# filtre_numerique=median_gaussian_trend "
                            f"median_span_mV={CV_TREND_MEDIAN_SPAN_MV:g} "
                            f"gaussian_span_mV={CV_TREND_GAUSSIAN_SPAN_MV:g} "
                            "par_branche=true donnees_brutes_preservees=true"
                        ])
                        writer.writerow([
                            "index", meta.get("xcol", "x"), "current_nA",
                            "current_uA", "current_filtered_nA",
                            "current_filtered_uA",
                        ])
                        for ((index, xval, current_na),
                             (_, _, filtered_na)) in zip(points, filtered_points):
                            writer.writerow([
                                index, f"{xval:.3f}", f"{current_na:.3f}",
                                f"{current_na / 1000.0:.6f}",
                                f"{filtered_na:.3f}",
                                f"{filtered_na / 1000.0:.6f}",
                            ])
                    else:
                        writer.writerow(["index", meta.get("xcol", "x"),
                                         "current_nA", "current_uA"])
                        for index, xval, current_na in points:
                            writer.writerow([
                                index, f"{xval:.3f}", f"{current_na:.3f}",
                                f"{current_na / 1000.0:.6f}",
                            ])
                paths["partial_csv"] = partial_path

            paths.update({"report": report_path, "json": json_path})
        except OSError as exc:
            paths["error"] = str(exc)
            self._log(f"⚠ Impossible d'écrire le rapport diagnostic : {exc}")
        return paths

    def _copy_journal(self):
        text_value = self.log.get("1.0", "end-1c")
        self.clipboard_clear()
        self.clipboard_append(text_value)
        self._set_status("Journal copié dans le presse-papiers.")

    def _save_journal(self):
        initial = f"ParisBioTek_journal_{datetime.now():%Y%m%d_%H%M%S}.log"
        path = filedialog.asksaveasfilename(
            defaultextension=".log", initialfile=initial,
            filetypes=[("Journal", "*.log"), ("Texte", "*.txt"), ("Tous", "*.*")],
        )
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as stream:
                stream.write(self.log.get("1.0", "end-1c"))
        except OSError as exc:
            messagebox.showerror("Journal", f"Échec de l'enregistrement :\n{exc}")
            return
        self._set_status(f"Journal enregistré : {os.path.basename(path)}")

    # ── Utilitaires ─────────────────────────────────────────────────────────────
    def _log(self, msg):
        now = datetime.now().astimezone()
        display_stamp = now.strftime("%H:%M:%S.%f")[:-3]
        message = str(msg)
        parts = message.splitlines() or [""]
        rendered = "\n".join(
            [f"{display_stamp} {parts[0]}"] +
            [f"             {line}" for line in parts[1:]]
        ) + "\n"
        self.log.config(state="normal")
        self.log.insert("end", rendered)
        if int(self.log.index("end-1c").split(".")[0]) > 2000:
            self.log.delete("1.0", "500.0")
        self.log.see("end")
        self.log.config(state="disabled")
        if self.log_file_path:
            try:
                with open(self.log_file_path, "a", encoding="utf-8") as stream:
                    stream.write(
                        now.isoformat(timespec="milliseconds") + " " +
                        message.replace("\n", "\n    ") + "\n"
                    )
            except OSError as exc:
                # Le widget reste fonctionnel même si le dossier devient
                # momentanément indisponible.
                self.log_file_path = None
                warning = (
                    f"{display_stamp} ⚠ Journal fichier désactivé : "
                    f"{type(exc).__name__}: {exc}\n"
                )
                self.log.config(state="normal")
                self.log.insert("end", warning)
                self.log.see("end")
                self.log.config(state="disabled")

    def _set_status(self, msg):
        self.status.set(msg)

    def _on_close(self):
        if self.busy:
            if not messagebox.askyesno(
                    "Mesure en cours",
                    "Une mesure est encore en cours. Fermer maintenant interrompra "
                    "le téléchargement. Voulez-vous vraiment fermer ?"):
                return
            context = {
                "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                "started_at": (self.last_run_dt.astimezone().isoformat(timespec="milliseconds")
                               if self.last_run_dt else None),
                "client_elapsed_ms": None,
                "exception_type": "OperatorClose",
                "message": "Fermeture de l'application demandée pendant la mesure.",
                "traceback": "(fermeture opérateur)",
                "params": self.last_params or {},
                "meta": self.last_meta or {},
                "start_reply": {},
                "last_status": self.last_status or {},
                "timeline": [],
                "points": list(self.points),
                "points_received": len(self.points),
                "transport": (self._active_transport.diagnostics_snapshot()
                              if self._active_transport else {}),
            }
            self._save_error_diagnostics(context)
        self._disconnect(force=True)
        self.destroy()


if __name__ == "__main__":
    CVApp().mainloop()
