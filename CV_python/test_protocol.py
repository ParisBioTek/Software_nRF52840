import base64
import queue
import struct
import threading
import unittest
from unittest.mock import patch

import main as app_main

from main import (
    METHOD_CV,
    SmpError,
    SmpProtocolError,
    SmpSerial,
    _firmware_measurement_blocker,
    _firmware_supports_stop,
    _flag_summary,
    _known_transport_failure_hint,
    _robust_filter_cv_points,
    _measurement_error_message,
    _status_timeline_entry,
    _status_summary,
    _validate_data_reply,
    _validate_release_reply,
    _validate_start_reply,
    _validate_status,
    _validate_stop_reply,
    cv_release,
    cv_stop,
    crc16_ccitt,
)


class DigitalFilterTests(unittest.TestCase):
    def test_robust_filter_removes_isolated_spikes_without_touching_raw_points(self):
        points = []
        for index in range(42):
            current = 0 if index < 21 else 1000
            if index == 10:
                current = 12000
            if index == 31:
                current = -9000
            voltage = index if index < 21 else 41 - index
            points.append((index, voltage, current))

        filtered = _robust_filter_cv_points(points, steps_per_cycle=42)

        self.assertEqual(points[10][2], 12000)
        self.assertAlmostEqual(filtered[10][2], 0, places=6)
        self.assertAlmostEqual(filtered[31][2], 1000, places=6)
        self.assertEqual([point[:2] for point in filtered],
                         [point[:2] for point in points])

    def test_filter_does_not_mix_forward_and_reverse_branches(self):
        points = [
            (index, index if index < 9 else 17 - index,
             0 if index < 9 else 10000)
            for index in range(18)
        ]
        filtered = _robust_filter_cv_points(points, steps_per_cycle=18)

        self.assertTrue(all(abs(point[2]) < 1e-6 for point in filtered[:9]))
        self.assertTrue(all(abs(point[2] - 10000) < 1e-6
                            for point in filtered[9:]))

    def test_trend_filter_smooths_a_broad_peak_without_shifting_it(self):
        points = [
            (index, index, 5000 - 4 * (index - 20) ** 2)
            for index in range(41)
        ] + [
            (41 + index, 40 - index, 1000 + 2 * (index - 20) ** 2)
            for index in range(41)
        ]

        filtered = _robust_filter_cv_points(points, steps_per_cycle=82)

        forward_peak = max(range(41), key=lambda pos: filtered[pos][2])
        reverse_peak = min(range(41, 82), key=lambda pos: filtered[pos][2])
        self.assertLessEqual(abs(forward_peak - 20), 1)
        self.assertLessEqual(abs(reverse_peak - 61), 1)
        self.assertLess(filtered[20][2], 5000)
        self.assertGreater(filtered[20][2], 4500)
        self.assertGreater(filtered[61][2], 1000)
        self.assertLess(filtered[61][2], 1250)

    def test_trend_filter_removes_a_three_sample_spike_burst(self):
        points = [(index, index, 1000.0) for index in range(50)] + [
            (50 + index, 49 - index, -1000.0) for index in range(50)
        ]
        for pos in (23, 24, 25):
            points[pos] = (points[pos][0], points[pos][1], 15000.0)

        filtered = _robust_filter_cv_points(points, steps_per_cycle=100)

        self.assertTrue(all(abs(filtered[pos][2] - 1000.0) < 1e-6
                            for pos in range(50)))


class GuiPreflightTests(unittest.TestCase):
    class Button:
        def __init__(self):
            self.state = None

        def config(self, **kwargs):
            self.state = kwargs.get("state", self.state)

    @patch("main.messagebox.showerror")
    def test_run_does_not_start_worker_with_pwr_v4(self, showerror):
        class Harness:
            busy = False
            smp = object()
            connected_status = {"fw_build": "20260824-pwr-v4"}

            def __init__(self):
                self.btn_run = GuiPreflightTests.Button()
                self.logs = []
                self.statuses = []

            def _log(self, message):
                self.logs.append(message)

            def _set_status(self, message):
                self.statuses.append(message)

            def _read_params(self):
                raise AssertionError("les paramètres ne doivent pas être lus")

        harness = Harness()
        app_main.CVApp._run(harness)

        self.assertEqual(harness.btn_run.state, "disabled")
        self.assertTrue(any("Mesure interdite" in item for item in harness.logs))
        showerror.assert_called_once()


class StartValidationTests(unittest.TestCase):
    def test_legacy_start_remains_compatible(self):
        reply = {"rc": 0, "steps": 800, "cycles": 1, "total_points": 800}
        self.assertIs(_validate_start_reply(reply), reply)

    def test_v4_start_requires_run_id(self):
        with self.assertRaises(SmpProtocolError):
            _validate_start_reply({
                "rc": 0, "diag_ver": 4, "release_required": True,
            })

    def test_v4_start_requires_release_true(self):
        with self.assertRaises(SmpProtocolError):
            _validate_start_reply({
                "rc": 0, "diag_ver": 4, "run_id": 7,
                "release_required": False,
            })

    def test_complete_v5_start_is_valid(self):
        reply = {
            "rc": 0, "diag_ver": 5, "run_id": 7,
            "release_required": True,
        }
        self.assertIs(_validate_start_reply(reply), reply)

    def test_v6_start_requires_confirmed_settling(self):
        with self.assertRaises(SmpProtocolError):
            _validate_start_reply({
                "rc": 0, "diag_ver": 6, "run_id": 7,
                "release_required": True,
            })

    def test_complete_v6_start_is_valid(self):
        reply = {
            "rc": 0, "diag_ver": 6, "run_id": 7,
            "release_required": True, "settle_ms": 2000,
        }
        self.assertIs(_validate_start_reply(reply), reply)


class StatusValidationTests(unittest.TestCase):
    def test_minimal_status_is_valid(self):
        status = {"run": True, "n": 12, "err": 0, "run_id": 7}
        self.assertIs(_validate_status(status, expected_points=800), status)

    def test_missing_run_is_rejected(self):
        with self.assertRaises(SmpProtocolError):
            _validate_status({"n": 0, "err": 0})

    def test_counter_regression_is_rejected(self):
        with self.assertRaises(SmpProtocolError):
            _validate_status(
                {"run": True, "n": 11, "err": 0, "run_id": 4},
                previous_n=12,
                previous_run_id=4,
            )

    def test_run_change_is_rejected(self):
        with self.assertRaises(SmpProtocolError):
            _validate_status(
                {"run": True, "n": 12, "err": 0, "run_id": 5},
                previous_run_id=4,
            )

    def test_run_id_cannot_disappear(self):
        with self.assertRaises(SmpProtocolError):
            _validate_status(
                {"run": True, "n": 12, "err": 0},
                previous_run_id=4,
            )

    @staticmethod
    def _valid_v4_status():
        return {
            "run": False, "n": 0, "err": -10, "diag_ver": 4,
            "run_id": 7, "data_held": True,
            "adiid": 0, "chipid": 0, "spi_mode": -1,
            "mode_ids01": 0, "mode_ids23": 0,
            "gpio_err": 0, "gpio_state": 0x701,
            "platform_err": -100, "spi_errors": 0, "spi_last": 0,
        }

    def test_complete_v4_status_is_valid(self):
        status = self._valid_v4_status()
        self.assertIs(_validate_status(status), status)

    def test_truncated_v4_status_is_rejected(self):
        status = self._valid_v4_status()
        del status["mode_ids23"]
        with self.assertRaises(SmpProtocolError):
            _validate_status(status)

    def test_wrong_v4_field_type_is_rejected(self):
        status = self._valid_v4_status()
        status["gpio_state"] = None
        with self.assertRaises(SmpProtocolError):
            _validate_status(status)

    def test_v4_spi_mode_range_is_checked(self):
        status = self._valid_v4_status()
        status["spi_mode"] = 4
        with self.assertRaises(SmpProtocolError):
            _validate_status(status)

    def test_v4_requires_run_id_and_data_lease_state(self):
        status = self._valid_v4_status()
        del status["data_held"]
        with self.assertRaises(SmpProtocolError):
            _validate_status(status)

    def test_complete_v5_status_is_valid(self):
        status = self._valid_v4_status()
        status.update({"diag_ver": 5, "mode_tried": 1, "hfxo_on": True})
        self.assertIs(_validate_status(status), status)

    def test_v5_requires_hfxo_state(self):
        status = self._valid_v4_status()
        status.update({"diag_ver": 5, "mode_tried": 1})
        with self.assertRaises(SmpProtocolError):
            _validate_status(status)

    def test_v5_mode_mask_is_limited_to_four_modes(self):
        status = self._valid_v4_status()
        status.update({"diag_ver": 5, "mode_tried": 0x10, "hfxo_on": True})
        with self.assertRaises(SmpProtocolError):
            _validate_status(status)

    def test_complete_v6_status_is_valid(self):
        status = self._valid_v4_status()
        status.update({
            "diag_ver": 6, "mode_tried": 1, "hfxo_on": True,
            "cancelled": False, "stop_requested": False, "settle_ms": 2000,
        })
        self.assertIs(_validate_status(status), status)

    def test_v6_requires_all_stop_contract_fields(self):
        status = self._valid_v4_status()
        status.update({
            "diag_ver": 6, "mode_tried": 1, "hfxo_on": True,
            "cancelled": False, "settle_ms": 2000,
        })
        with self.assertRaises(SmpProtocolError):
            _validate_status(status)

    def test_v6_cancelled_status_has_strict_invariants(self):
        status = self._valid_v4_status()
        status.update({
            "diag_ver": 6, "mode_tried": 1, "hfxo_on": True,
            "run": False, "n": 12, "err": 0, "stage": "cancelled",
            "cancelled": True, "stop_requested": True, "settle_ms": 2000,
        })
        self.assertIs(_validate_status(status), status)
        status["run"] = True
        with self.assertRaises(SmpProtocolError):
            _validate_status(status)


class DataValidationTests(unittest.TestCase):
    def test_valid_batch(self):
        n, pairs = _validate_data_reply(
            {"run_id": 3, "off": 2, "n": 4, "v": [10, 11], "i": [-2, -3]},
            2,
            run_id=3,
        )
        self.assertEqual(n, 4)
        self.assertEqual(pairs, [(10.0, -2.0), (11.0, -3.0)])

    def test_wrong_offset_is_rejected(self):
        with self.assertRaises(SmpProtocolError):
            _validate_data_reply({"off": 1, "n": 2, "v": [1], "i": [2]}, 0)

    def test_unequal_lists_are_rejected(self):
        with self.assertRaises(SmpProtocolError):
            _validate_data_reply({"off": 0, "n": 2, "v": [1, 2], "i": [3]}, 0)

    def test_stale_run_is_rejected(self):
        with self.assertRaises(SmpProtocolError):
            _validate_data_reply(
                {"run_id": 8, "off": 0, "n": 1, "v": [1], "i": [2]},
                0,
                run_id=9,
            )

    def test_data_run_id_is_required_when_session_is_known(self):
        with self.assertRaises(SmpProtocolError):
            _validate_data_reply(
                {"off": 0, "n": 1, "v": [1], "i": [2]},
                0,
                run_id=9,
            )


class ReleaseValidationTests(unittest.TestCase):
    def test_release_sends_run_and_point_count(self):
        class FakeTransport:
            def request(self, op, group, cmd, payload):
                self.call = (op, group, cmd, payload)
                return {"rc": 0, "run_id": 9, "n": 233, "released": True}

        transport = FakeTransport()
        reply = cv_release(transport, 9, 233)
        self.assertEqual(transport.call, (2, 64, 3, {"run_id": 9, "n": 233}))
        self.assertTrue(reply["released"])

    def test_valid_release_ack(self):
        self.assertIsNone(_validate_release_reply(
            {"rc": 0, "run_id": 9, "n": 233, "released": True}, 9, 233
        ))

    def test_release_cannot_ack_another_point_count(self):
        with self.assertRaises(SmpProtocolError):
            _validate_release_reply(
                {"rc": 0, "run_id": 9, "n": 232, "released": True}, 9, 233
            )

    def test_release_requires_explicit_true(self):
        with self.assertRaises(SmpProtocolError):
            _validate_release_reply(
                {"rc": 0, "run_id": 9, "n": 233, "released": 1}, 9, 233
            )


class StopValidationTests(unittest.TestCase):
    def test_stop_sends_only_the_run_identity(self):
        class FakeTransport:
            def request(self, op, group, cmd, payload):
                self.call = (op, group, cmd, payload)
                return {
                    "rc": 0, "run_id": 9, "n": 233,
                    "stop_requested": True,
                }

        transport = FakeTransport()
        reply = cv_stop(transport, 9)
        self.assertEqual(transport.call, (2, 64, 4, {"run_id": 9}))
        self.assertTrue(reply["stop_requested"])

    def test_valid_stop_ack(self):
        reply = {
            "rc": 0, "run_id": 9, "n": 233,
            "stop_requested": True,
        }
        self.assertIs(_validate_stop_reply(reply, 9), reply)

    def test_stop_ack_rejects_wrong_run(self):
        with self.assertRaises(SmpProtocolError):
            _validate_stop_reply({
                "rc": 0, "run_id": 8, "n": 233,
                "stop_requested": True,
            }, 9)

    def test_stop_capability_begins_at_diag_v6(self):
        self.assertFalse(_firmware_supports_stop({"diag_ver": 5}))
        self.assertTrue(_firmware_supports_stop({"diag_ver": 6}))


class WorkerLeaseTests(unittest.TestCase):
    class Harness:
        def __init__(self):
            self.q = queue.Queue()

    class FakeTransport:
        port = "COM_TEST"

        def __init__(self, statuses, points):
            self.statuses = list(statuses)
            self.last_status = dict(statuses[-1])
            self.points = points
            self.calls = []

        def request(self, op, group, cmd, payload):
            self.calls.append((op, group, cmd, dict(payload)))
            if cmd == 0:
                return {
                    "rc": 0, "run_id": 7, "release_required": True,
                    "cycles": 1, "steps": 2, "total_points": 2,
                }
            if cmd == 1:
                if self.statuses:
                    self.last_status = dict(self.statuses.pop(0))
                return dict(self.last_status)
            if cmd == 2:
                off = payload["off"]
                batch = self.points[off:off + 64]
                return {
                    "run_id": 7, "off": off, "n": len(self.points),
                    "v": [p[0] for p in batch], "i": [p[1] for p in batch],
                }
            if cmd == 3:
                return {
                    "rc": 0, "run_id": payload["run_id"], "n": payload["n"],
                    "released": True,
                }
            raise AssertionError(f"commande inattendue {cmd}")

        def diagnostics_snapshot(self):
            return {"port": self.port, "calls": len(self.calls)}

    @staticmethod
    def params_meta():
        return (
            {"method": METHOD_CV, "steps": 2, "cycles": 1, "rtia": 4000},
            {"title": "CV test", "dur_ms": 100, "expected_points": 2},
        )

    @staticmethod
    def v6_status(**changes):
        status = StatusValidationTests._valid_v4_status()
        status.update({
            "diag_ver": 6, "mode_tried": 1, "hfxo_on": True,
            "run_id": 7, "run": False, "n": 0, "err": 0,
            "stage": "complete", "cancelled": False,
            "stop_requested": False, "settle_ms": 0,
        })
        status.update(changes)
        return status

    @patch("main.time.sleep", return_value=None)
    def test_success_releases_after_final_status(self, _sleep):
        status = {"run_id": 7, "run": False, "n": 2, "err": 0}
        transport = self.FakeTransport([status, status], [(10, 20), (11, 21)])
        harness = self.Harness()
        params, meta = self.params_meta()

        app_main.CVApp._run_worker(harness, transport, params, meta)

        events = list(harness.q.queue)
        self.assertTrue(any(kind == "run_done" for kind, _ in events))
        release_log_index = next(
            i for i, event in enumerate(events)
            if event[0] == "log" and "Buffer firmware libéré" in event[1]
        )
        done_index = next(i for i, event in enumerate(events) if event[0] == "run_done")
        self.assertLess(release_log_index, done_index)
        release_calls = [call for call in transport.calls if call[2] == 3]
        self.assertEqual(release_calls[-1][3], {"run_id": 7, "n": 2})
        self.assertGreater(
            max(i for i, call in enumerate(transport.calls) if call[2] == 1),
            min(i for i, call in enumerate(transport.calls) if call[2] == 2),
        )

    @patch("main.time.sleep", return_value=None)
    def test_transient_status_timeout_at_cycle_end_is_retried(self, _sleep):
        class OneLostStatusTransport(self.FakeTransport):
            def __init__(inner_self, statuses, points):
                super().__init__(statuses, points)
                inner_self.status_timeout_sent = False

            def request(inner_self, op, group, cmd, payload):
                if cmd == 1 and not inner_self.status_timeout_sent:
                    inner_self.status_timeout_sent = True
                    inner_self.calls.append((op, group, cmd, dict(payload)))
                    raise SmpError(
                        "pas de réponse SMP (timeout) [groupe=64, cmd=1, seq=9]"
                    )
                return super(OneLostStatusTransport, inner_self).request(
                    op, group, cmd, payload
                )

        status = {"run_id": 7, "run": False, "n": 2, "err": 0,
                  "stage": "complete"}
        transport = OneLostStatusTransport(
            [status, status], [(10, 20), (11, 21)]
        )
        harness = self.Harness()
        params, meta = self.params_meta()

        app_main.CVApp._run_worker(harness, transport, params, meta)

        events = list(harness.q.queue)
        self.assertTrue(any(kind == "run_done" for kind, _ in events))
        self.assertFalse(any(kind == "run_err" for kind, _ in events))
        self.assertTrue(any(
            kind == "log" and "STATUS timeout transitoire" in payload
            for kind, payload in events
        ))

    @patch("main.time.sleep", return_value=None)
    def test_release_uses_newer_data_counter_than_last_status(self, _sleep):
        # Reproduit le journal terrain : STATUS voit n=1, mais DATA a déjà vu
        # le point suivant. RELEASE doit acquitter 2, pas l'ancien compteur 1.
        status = {"run_id": 7, "run": False, "n": 1, "err": 0}
        transport = self.FakeTransport([status, status], [(10, 20), (11, 21)])
        harness = self.Harness()
        params, meta = self.params_meta()

        app_main.CVApp._run_worker(harness, transport, params, meta)

        release_calls = [call for call in transport.calls if call[2] == 3]
        self.assertEqual(release_calls[-1][3], {"run_id": 7, "n": 2})
        self.assertTrue(any(kind == "run_done" for kind, _ in harness.q.queue))

    @patch("main.time.sleep", return_value=None)
    def test_user_stop_keeps_partial_points_and_releases(self, _sleep):
        class StopTransport(self.FakeTransport):
            def request(inner_self, op, group, cmd, payload):
                if cmd == 0:
                    inner_self.calls.append((op, group, cmd, dict(payload)))
                    return {
                        "rc": 0, "diag_ver": 6, "run_id": 7,
                        "release_required": True, "settle_ms": 0,
                        "cycles": 1, "steps": 2, "total_points": 2,
                    }
                if cmd == 4:
                    inner_self.calls.append((op, group, cmd, dict(payload)))
                    return {
                        "rc": 0, "run_id": 7, "n": 1,
                        "stop_requested": True,
                    }
                return super(StopTransport, inner_self).request(
                    op, group, cmd, payload
                )

        cancelled = self.v6_status(
            n=1, stage="cancelled", cancelled=True, stop_requested=True,
        )
        transport = StopTransport([cancelled, cancelled], [(10, 20)])
        harness = self.Harness()
        params, meta = self.params_meta()
        params["settle_ms"] = 0
        stop_event = threading.Event()
        stop_event.set()

        app_main.CVApp._run_worker(
            harness, transport, params, meta, stop_event
        )

        events = list(harness.q.queue)
        stopped = [payload for kind, payload in events if kind == "run_stopped"]
        self.assertEqual(len(stopped), 1)
        self.assertEqual(len(stopped[0][0]), 1)
        self.assertTrue(any(call[2] == 4 for call in transport.calls))
        self.assertTrue(any(call[2] == 3 and call[3]["n"] == 1
                            for call in transport.calls))
        self.assertFalse(any(kind == "run_err" for kind, _ in events))

    @patch("main.time.sleep", return_value=None)
    def test_firmware_error_keeps_primary_error_and_releases_partial(self, _sleep):
        running = {"run_id": 7, "run": True, "n": 1, "err": -24, "stage": "cv/deadline"}
        stopped = {"run_id": 7, "run": False, "n": 1, "err": -24, "stage": "cv/deadline"}
        transport = self.FakeTransport([running, stopped], [(10, 20)])
        harness = self.Harness()
        params, meta = self.params_meta()

        app_main.CVApp._run_worker(harness, transport, params, meta)

        events = list(harness.q.queue)
        errors = [payload for kind, payload in events if kind == "run_err"]
        self.assertEqual(len(errors), 1)
        self.assertIn("-24", errors[0]["message"])
        self.assertEqual(errors[0]["points_received"], 1)
        self.assertTrue(errors[0]["release"]["released"])
        self.assertTrue(any(call[2] == 3 and call[3]["n"] == 1
                            for call in transport.calls))

    @patch("main.time.sleep", return_value=None)
    def test_v4_start_without_run_id_cannot_publish_success(self, _sleep):
        class BrokenStartTransport(self.FakeTransport):
            def request(inner_self, op, group, cmd, payload):
                if cmd == 0:
                    inner_self.calls.append((op, group, cmd, dict(payload)))
                    return {
                        "rc": 0, "diag_ver": 4,
                        "release_required": True,
                        "cycles": 1, "steps": 2, "total_points": 2,
                    }
                return super(BrokenStartTransport, inner_self).request(
                    op, group, cmd, payload
                )

        status = {"run_id": 7, "run": False, "n": 2, "err": 0}
        transport = BrokenStartTransport([status], [(10, 20), (11, 21)])
        harness = self.Harness()
        params, meta = self.params_meta()

        app_main.CVApp._run_worker(harness, transport, params, meta)

        events = list(harness.q.queue)
        self.assertFalse(any(kind == "run_done" for kind, _ in events))
        errors = [payload for kind, payload in events if kind == "run_err"]
        self.assertEqual(len(errors), 1)
        self.assertIn("run_id", errors[0]["message"])

    @patch("main.time.monotonic", side_effect=[0, 1, 2, 3, 100, 101, 102])
    @patch("main.time.sleep", return_value=None)
    def test_timeout_keeps_reported_firmware_error(self, _sleep, _monotonic):
        running = {"run_id": 7, "run": True, "n": 1, "err": -24,
                   "stage": "cv/deadline"}
        transport = self.FakeTransport([running], [(10, 20)])
        harness = self.Harness()
        params, meta = self.params_meta()

        app_main.CVApp._run_worker(harness, transport, params, meta)

        errors = [payload for kind, payload in harness.q.queue if kind == "run_err"]
        self.assertEqual(len(errors), 1)
        self.assertIn("Code err=-24", errors[0]["message"])
        self.assertIn("délai client", errors[0]["message"])


class DiagnosticFormattingTests(unittest.TestCase):
    def test_zero_interrupt_flags_are_explicitly_harmless(self):
        self.assertEqual(_flag_summary(0), "0x00000000 (aucun)")

    def test_old_233_point_failure_identifies_lfosc_signature(self):
        message = _measurement_error_message(
            {"run": False, "n": 233, "err": -1, "method": METHOD_CV,
             "steps": 800, "cycles": 1, "total_points": 800},
            {"method": METHOD_CV, "steps": 800, "cycles": 1, "rtia": 4000},
            {"dur_ms": 18000, "expected_points": 800},
            {"steps": 800, "cycles": 1, "total_points": 800},
        )
        self.assertIn("LFOSC", message)
        self.assertIn("146", message)
        self.assertIn("233/800", message)

    def test_v2_status_decodes_fatal_flag(self):
        summary = _status_summary({
            "diag_ver": 2, "run": False, "n": 233, "err": -22,
            "intc0": 0, "intc1": 0x00400000, "int_fatal": 0x00400000,
        })
        self.assertIn("CMDFIFO_UNDERFLOW", summary)

    def test_sleep_key_signature_is_not_labelled_as_timeout(self):
        summary = _status_summary({
            "diag_ver": 2, "run": False, "n": 33, "err": -22,
            "intc0": 0x000A47E5, "intc1": 0x41004021,
            "int_fatal": 0x00020000,
        })
        self.assertIn("LECTURE_PÉRIMÉE", summary)

        message = _measurement_error_message(
            {"diag_ver": 2, "run": False, "n": 33, "err": -22,
             "method": METHOD_CV, "intc0": 0x000A47E5,
             "intc1": 0x41004021, "int_fatal": 0x00020000},
            {"method": METHOD_CV, "steps": 800, "cycles": 1, "rtia": 1000},
            {"dur_ms": 22000, "expected_points": 800},
            {"steps": 800, "cycles": 1, "total_points": 800},
        )
        self.assertIn("Cause identifiée", message)
        self.assertIn("SEQSLPLOCK", message)

    def test_v3_status_shows_rechecks_and_sequence_registers(self):
        summary = _status_summary({
            "diag_ver": 3, "run": True, "n": 10, "err": 0,
            "intc0": 0, "intc1": 0, "int_fatal": 0,
            "intc_stale": 1, "intc_retries": 1,
            "fatal_rechecks": 1, "fatal_unconfirmed": 1,
            "intc0_rejected": 0x000A47E5, "intc1_rejected": 0,
            "seq_con": 0x11, "seq_count": 42, "seq_timeout": 0,
            "seq0_info": 1, "seq1_info": 2, "seq2_info": 3,
        })
        self.assertIn("Intégrité INTC", summary)
        self.assertIn("SEQCON=0x00000011", summary)

    def test_init_sequence_diagnostic_is_reported(self):
        summary = _status_summary({
            "diag_ver": 6, "run": False, "n": 0, "err": -20,
            "intc0": 0, "intc1": 0, "int_fatal": 0,
            "init_wait_polls": 2, "init_endseq_missing": 1,
            "init_direct_fallback": 1,
            "seq3_info": 0x00000044,
        })
        self.assertIn("Initialisation sequence", summary)
        self.assertIn("ENDSEQ absent mais SEQ_STOP confirme=1", summary)
        self.assertIn("repli MCU direct=oui", summary)

        message = _measurement_error_message(
            {"diag_ver": 6, "run": False, "n": 0, "err": -20,
             "err_detail": -10, "init_wait_polls": 50,
             "init_endseq_missing": 0, "init_direct_fallback": 1,
             "seq3_info": 0},
            {"method": METHOD_CV, "steps": 800, "cycles": 1, "rtia": 4000},
            {"dur_ms": 20000, "expected_points": 800},
            {"steps": 800, "cycles": 1, "total_points": 800},
        )
        self.assertIn("timeout interne AD5940 pendant cv/init", message)
        self.assertIn("même configuration a été appliquée directement par SPI", message)

    def test_v4_status_decodes_sensor_probe_and_gpio_latches(self):
        summary = _status_summary({
            "diag_ver": 4, "run": False, "n": 0, "err": -10,
            "adiid": 0, "chipid": 0, "spi_mode": -1,
            "mode_ids01": 0, "mode_ids23": 0,
            "gpio_err": 0, "gpio_state": 0x701,
            "intc0": 0, "intc1": 0, "int_fatal": 0,
        })
        self.assertIn("Sonde matérielle v4", summary)
        self.assertIn("M0=0x0000/M1=0x0000/M2=0x0000/M3=0x0000", summary)
        self.assertIn("latch PWR/CS/RESET=H/H/H", summary)

    def test_v4_silent_sensor_is_distinguished_from_usb_and_cv(self):
        message = _measurement_error_message(
            {"fw_build": "20260824-pwr-v4", "diag_ver": 4,
             "run": False, "n": 0, "err": -10, "method": METHOD_CV,
             "adiid": 0, "chipid": 0, "spi_mode": -1,
             "mode_ids01": 0, "mode_ids23": 0,
             "gpio_err": 0, "gpio_state": 0x701,
             "spi_errors": 0, "spi_last": 0},
            {"method": METHOD_CV, "steps": 800, "cycles": 1, "rtia": 4000},
            {"dur_ms": 24000, "expected_points": 800},
            {"steps": 800, "cycles": 1, "total_points": 800},
        )
        self.assertIn("Cause isolée", message)
        self.assertIn("Ce n'est pas une panne USB/SMP", message)
        self.assertIn("DVDD/IOVDD", message)

    def test_v4_all_ffff_does_not_claim_untried_modes_are_silent(self):
        message = _measurement_error_message(
            {"fw_build": "20260824-pwr-v4", "diag_ver": 4,
             "run": False, "n": 0, "err": -10, "method": METHOD_CV,
             "adiid": 0xFFFF, "chipid": 0xFFFF, "spi_mode": -1,
             "mode_ids01": 0xFFFFFFFF, "mode_ids23": 0xFFFFFFFF,
             "gpio_err": 0, "gpio_state": 0x701,
             "spi_errors": 0, "spi_last": 0},
            {"method": METHOD_CV, "steps": 800, "cycles": 1, "rtia": 4000},
            {"dur_ms": 24000, "expected_points": 800},
            {"steps": 800, "cycles": 1, "total_points": 800},
        )
        self.assertNotIn("Cause isolée", message)

    def test_v5_summary_distinguishes_tried_and_untried_spi_modes(self):
        summary = _status_summary({
            "diag_ver": 5, "run_id": 7, "run": True, "data_held": False,
            "n": 10, "err": 0, "adiid": 0x4144, "chipid": 0x5502,
            "spi_mode": 0, "mode_ids01": 0xFFFF4144,
            "mode_ids23": 0xFFFFFFFF, "mode_tried": 0x1,
            "hfxo_on": True, "gpio_err": 0, "gpio_state": 0x701,
        })
        self.assertIn("Sonde matérielle v5", summary)
        self.assertIn("M0=0x4144/M1=non essayé", summary)
        self.assertIn("modes essayés=0x1", summary)
        self.assertIn("OE_OSC1_commandé=ON", summary)

    def test_v5_partial_mode_mask_reports_only_modes_actually_tried(self):
        message = _measurement_error_message(
            {"fw_build": "20260824-clk-v5", "diag_ver": 5,
             "run": False, "n": 0, "err": -10, "method": METHOD_CV,
             "adiid": 0, "chipid": 0, "spi_mode": -1,
             "mode_ids01": 0xFFFF0000, "mode_ids23": 0xFFFFFFFF,
             "mode_tried": 0x1, "hfxo_on": True,
             "gpio_err": 0, "gpio_state": 0x701,
             "spi_errors": 0, "spi_last": 0},
            {"method": METHOD_CV, "steps": 800, "cycles": 1, "rtia": 4000},
            {"dur_ms": 24000, "expected_points": 800},
            {"steps": 800, "cycles": 1, "total_points": 800},
        )
        self.assertIn("tous les modes SPI réellement essayés", message)
        self.assertNotIn("quatre modes SPI", message)

    def test_pwr_v4_near_end_write_failure_has_known_clock_hint(self):
        hint = _known_transport_failure_hint(
            SmpError("délai d'écriture expiré", {
                "last_exchange": {"error_stage": "serial/write"},
            }),
            {"fw_build": "20260824-pwr-v4", "run": True,
             "n": 792, "err": 0},
            800,
        )
        self.assertIn("P0.08", hint)
        self.assertIn("20260824-clk-v5", hint)

    def test_protocol_error_never_gets_clock_transport_hint(self):
        hint = _known_transport_failure_hint(
            SmpProtocolError("STATUS tronqué", {
                "last_exchange": {"error_stage": "serial/write"},
            }),
            {"fw_build": "20260824-pwr-v4", "run": True,
             "n": 792, "err": 0},
            800,
        )
        self.assertEqual(hint, "")

    def test_clock_hint_requires_a_recorded_serial_failure(self):
        hint = _known_transport_failure_hint(
            SmpError("erreur client"),
            {"fw_build": "20260824-pwr-v4", "run": True,
             "n": 792, "err": 0},
            800,
        )
        self.assertEqual(hint, "")

    def test_known_bad_firmware_is_blocked_before_measurement(self):
        message = _firmware_measurement_blocker({
            "fw_build": "20260824-pwr-v4", "diag_ver": 4,
        })
        self.assertIn("Mesure interdite", message)
        self.assertIn("20260825-directinit-v8", message)
        self.assertEqual(_firmware_measurement_blocker({
            "fw_build": "20260824-clk-v5", "diag_ver": 5,
        }), "")

    def test_timeline_keeps_v5_transport_and_firmware_error_fields(self):
        entry = _status_timeline_entry({
            "run": True, "n": 792, "err": 0,
            "error_stage": "none", "elapsed_ms": 19980,
            "platform_err": 0, "spi_last": 0,
            "lfosc_err": 0, "lfosc_fallback": False,
            "mode_tried": 1, "hfxo_on": True,
        }, 20123.6)
        self.assertEqual(entry["client_elapsed_ms"], 20124)
        for key in ("error_stage", "elapsed_ms", "platform_err", "spi_last",
                    "lfosc_err", "lfosc_fallback", "mode_tried", "hfxo_on"):
            self.assertIn(key, entry)

    def test_timeline_keeps_v6_stop_and_settling_fields(self):
        entry = _status_timeline_entry({
            "run": False, "n": 42, "err": 0,
            "cancelled": True, "stop_requested": True, "settle_ms": 2000,
        }, 123.4)
        self.assertTrue(entry["cancelled"])
        self.assertTrue(entry["stop_requested"])
        self.assertEqual(entry["settle_ms"], 2000)

    def test_v3_platform_failure_mentions_dk_qspi_regression(self):
        message = _measurement_error_message(
            {"fw_build": "20260824-intc-v3", "diag_ver": 3,
             "run": False, "n": 0, "err": -10, "method": METHOD_CV,
             "adiid": 0, "spi_errors": 0},
            {"method": METHOD_CV, "steps": 800, "cycles": 1, "rtia": 4000},
            {"dur_ms": 24000, "expected_points": 800},
            {"steps": 800, "cycles": 1, "total_points": 800},
        )
        self.assertIn("QSPI", message)
        self.assertIn("P0.17/P0.20/P0.22", message)


class FramingTests(unittest.TestCase):
    def test_serial_frame_contains_length_and_valid_crc(self):
        packet = b"\x00\x00\x00\x01\x00\x40\x01\x02" + b"x" * 120
        framed = SmpSerial._frame(packet)
        raw = b""
        for line in framed.splitlines():
            raw += base64.b64decode(line[2:])
        announced = struct.unpack(">H", raw[:2])[0]
        body = raw[2:2 + announced]
        self.assertEqual(body[:-2], packet)
        self.assertEqual(struct.unpack(">H", body[-2:])[0], crc16_ccitt(packet))


class TransportDiagnosticsTests(unittest.TestCase):
    class FakeSerial:
        is_open = True
        in_waiting = 0
        out_waiting = 0

        def __init__(self, *, write_exc=None, read_exc=None):
            self.write_exc = write_exc
            self.read_exc = read_exc

        def reset_input_buffer(self):
            pass

        def write(self, frame):
            if self.write_exc is not None:
                raise self.write_exc
            return len(frame)

        def readline(self):
            if self.read_exc is not None:
                raise self.read_exc
            return b""

        def close(self):
            self.is_open = False

    def test_write_timeout_records_command_and_serial_state(self):
        fake = self.FakeSerial(
            write_exc=app_main.serial.SerialTimeoutException("Write timeout")
        )
        with patch("main.serial.Serial", return_value=fake):
            client = SmpSerial("COM_TEST")
        with self.assertRaises(SmpError) as raised:
            client.request(0, 64, 1, {})

        self.assertIn("groupe=64, cmd=1, seq=0", str(raised.exception))
        diag = raised.exception.context
        self.assertEqual(diag["stats"]["write_timeouts"], 1)
        self.assertEqual(diag["stats"]["tx_bytes"], 0)
        self.assertGreater(diag["stats"]["tx_attempted_bytes"], 0)
        self.assertEqual(diag["last_exchange"]["error_stage"], "serial/write")
        self.assertTrue(diag["serial_state"]["is_open"])

    def test_read_driver_error_is_classified_separately(self):
        fake = self.FakeSerial(
            read_exc=app_main.serial.SerialException("ClearCommError failed")
        )
        with patch("main.serial.Serial", return_value=fake):
            client = SmpSerial("COM_TEST")
        with self.assertRaises(SmpError) as raised:
            client.request(0, 64, 1, {})

        diag = raised.exception.context
        self.assertEqual(diag["stats"]["read_errors"], 1)
        self.assertEqual(diag["last_exchange"]["error_stage"], "serial/read")
        self.assertIn("lecture série impossible", str(raised.exception))


if __name__ == "__main__":
    unittest.main()
