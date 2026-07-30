from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np
import cv2


PI_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = PI_ROOT.parent
sys.path.insert(0, str(PI_ROOT))

from f2026_vision.geometry import ScopeLocator
from f2026_vision.phase import PhaseFrequencyTracker
from f2026_vision.pipeline import VisionPipeline
from f2026_vision.serial_link import RecordingMcuLink


def _load_simulator():
    path = PROJECT_ROOT / "q5_sawtooth_vision_sim" / "vision_sim.py"
    specification = importlib.util.spec_from_file_location("q5_vision_sim", path)
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


SIM = _load_simulator()


class VisionPipelineTests(unittest.TestCase):
    def _camera_image(self, frequency_hz: float, ramp_seconds: float, seed: int):
        config = SIM.replace(
            SIM.SimulationConfig(),
            ramp_seconds=ramp_seconds,
            accumulated_frames=1,
        )
        scope = SIM.draw_scope(frequency_hz, config, seed)
        camera, _ = SIM.camera_warp(scope)
        return camera

    def _trace_free_camera_image(self):
        config = SIM.SimulationConfig()
        image = np.zeros((config.height, config.width, 3), dtype=np.uint8)
        image[:] = (7, 10, 7)
        for index in range(11):
            x = round(config.scope_left + index * (config.scope_right - config.scope_left) / 10)
            color = (54, 61, 54) if index == 5 else (35, 43, 35)
            cv2.line(image, (x, config.scope_top), (x, config.scope_bottom), color, 1, cv2.LINE_AA)
        for index in range(9):
            y = round(config.scope_top + index * (config.scope_bottom - config.scope_top) / 8)
            color = (54, 61, 54) if index == 4 else (35, 43, 35)
            cv2.line(image, (config.scope_left, y), (config.scope_right, y), color, 1, cv2.LINE_AA)
        camera, _ = SIM.camera_warp(image)
        return camera

    def _calibrated_pipeline(self):
        background = self._trace_free_camera_image()
        pipeline = VisionPipeline()
        pipeline.calibrate([background])
        return pipeline

    def test_locator_and_frequency_at_37_4_khz(self):
        camera = self._camera_image(37400.0, 0.001, 7)
        measurement = self._calibrated_pipeline().process(camera, 0.001)
        self.assertLess(abs(measurement.frequency.frequency_hz - 37400.187), 35.0)
        self.assertEqual(measurement.frequency.nominal_100hz, 37400)
        self.assertGreater(measurement.trace.valid_fraction, 0.90)

    def test_high_range_uses_half_millisecond_probe(self):
        camera = self._camera_image(100000.0, 0.0005, 11)
        measurement = self._calibrated_pipeline().process(camera, 0.0005)
        self.assertLess(abs(measurement.frequency.frequency_hz - 100000.5), 45.0)
        self.assertEqual(measurement.frequency.nominal_100hz, 100000)


class PhaseTrackerTests(unittest.TestCase):
    def test_two_hz_residual_across_wrapped_phase(self):
        tracker = PhaseFrequencyTracker(100000.0)
        timestamps = np.arange(45, dtype=np.float64) / 30.0
        rng = np.random.default_rng(123)
        phases = 0.4 + 2.0 * np.pi * 2.0 * timestamps
        phases += rng.normal(0.0, np.deg2rad(0.25), phases.size)
        phases = (phases + np.pi) % (2.0 * np.pi) - np.pi
        for timestamp, phase in zip(timestamps, phases):
            tracker.add(float(timestamp), float(phase))
        result = tracker.estimate()
        self.assertIsNotNone(result)
        self.assertLess(abs(result.residual_hz - 2.0), 0.01)


class SerialProtocolTests(unittest.TestCase):
    def test_current_stm32_commands(self):
        link = RecordingMcuLink()
        link.set_frequency_hz(100002.4)
        link.set_phase_degrees(-90.0)
        link.set_auto_mode("circle")
        self.assertEqual(
            link.commands,
            ["FREQ 100002", "PHASE 270", "AUTO CIRCLE"],
        )


if __name__ == "__main__":
    unittest.main()
