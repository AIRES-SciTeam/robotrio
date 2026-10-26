"""Run with: python3 -m unittest discover -s dog/onboard/tests -v."""

import sys
import unittest
from pathlib import Path
from types import ModuleType
from unittest.mock import Mock, patch


# Import without installing the hardware SDK or opening a serial connection.
sdk = ModuleType("xgolib")
sdk.XGO_DOG = Mock(side_effect=AssertionError("Hardware must not be initialized"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
try:
    with patch.dict(sys.modules, {"xgolib": sdk}):
        from robotrio_dog.commander import Commander
finally:
    sys.path.pop(0)


class ValidatorTests(unittest.TestCase):
    def assert_validation(self, validator, value, expected, *args):
        result = validator(value, *args)
        self.assertIsInstance(result, tuple)
        self.assertEqual(len(result), 2)
        valid, message = result
        self.assertIs(valid, expected)
        if not valid:
            self.assertIsInstance(message, str)
            self.assertTrue(message.strip())

    def test_vector_shapes_and_element_types(self):
        for validator, size in (
            (Commander._valid_lin_vel, 2),
            (Commander._valid_translation, 3),
            (Commander._valid_rotation, 3),
        ):
            bad_shapes = [None, 0, "0" * size, [0] * size, {}, (),
                          (0,) * (size - 1), (0,) * (size + 1)]
            for relative in (False, True):
                for value in bad_shapes:
                    with self.subTest(validator=validator.__name__, value=value, relative=relative):
                        self.assert_validation(validator, value, False, relative)
                for axis in range(size):
                    for bad in ("1", [], {}, object(), 1j):
                        value = [None] * size
                        value[axis] = bad
                        with self.subTest(validator=validator.__name__, axis=axis, bad=bad, relative=relative):
                            self.assert_validation(validator, tuple(value), False, relative)

    def test_absolute_vector_boundaries(self):
        for validator, baseline, ranges in (
            (Commander._valid_lin_vel, (0, 0), ((-25, 25), (-18, 18))),
            (Commander._valid_translation, (0, 0, 85), ((-25, 25), (-18, 18), (60, 110))),
            (Commander._valid_rotation, (0, 0, 0), ((-20, 20), (-22, 22), (-16, 16))),
        ):
            for axis, (low, high) in enumerate(ranges):
                for component, expected in (
                    (low, True), (high, True), ((low + high) / 2, True),
                    (low - 0.01, False), (high + 0.01, False),
                    (float("nan"), False), (float("inf"), False), (-float("inf"), False),
                ):
                    value = list(baseline)
                    value[axis] = component
                    with self.subTest(validator=validator.__name__, axis=axis, component=component):
                        self.assert_validation(validator, tuple(value), expected, False)

    def test_relative_vector_boundaries(self):
        for validator, size in (
            (Commander._valid_lin_vel, 2),
            (Commander._valid_translation, 3),
            (Commander._valid_rotation, 3),
        ):
            for axis in range(size):
                for component, expected in (
                    (None, True), (-100, True), (100, True), (0, True), (12.5, True),
                    (-100.01, False), (100.01, False), (float("nan"), False),
                    (float("inf"), False), (-float("inf"), False),
                ):
                    value = [None] * size
                    value[axis] = component
                    with self.subTest(validator=validator.__name__, axis=axis, component=component):
                        self.assert_validation(validator, tuple(value), expected, True)

    def test_optional_absolute_vector_components(self):
        for validator, values in (
            (Commander._valid_lin_vel, [(None, None), (None, 18), (-25, None)]),
            (Commander._valid_rotation, [(None, None, None), (None, 22, None), (20, None, -16)]),
            (Commander._valid_translation, [(None, None, 60), (25, None, 110), (None, -18, 85.5)]),
        ):
            for value in values:
                with self.subTest(validator=validator.__name__, value=value):
                    self.assert_validation(validator, value, True, False)

    def test_angular_velocity(self):
        for relative in (False, True):
            for value, expected in (
                (None, True), (0, True), (-100, True), (100, True), (0.5, True),
                (-100.01, False), (100.01, False), (float("nan"), False),
                (float("inf"), False), (-float("inf"), False),
                ("1", False), ((0,), False), ([], False), ({}, False), (1j, False),
            ):
                with self.subTest(value=value, relative=relative):
                    self.assert_validation(Commander._valid_ang_vel, value, expected, relative)

    def test_time_limit(self):
        for value, expected in (
            (None, True), (0, True), (0.0, True), (0.01, True), (10, True),
            (-0.01, False), (-1, False), (float("nan"), False),
            (-float("inf"), False), ("10", False), ([], False), ({}, False), (1j, False),
        ):
            with self.subTest(value=value):
                self.assert_validation(Commander._valid_approx_time_limit, value, expected)

    def test_gait_names(self):
        for gait in ("slow_trot", "trot", "walk", "high_walk"):
            for value in (gait, gait.upper(), gait.capitalize()):
                with self.subTest(value=value):
                    self.assert_validation(Commander._valid_gait, value, True)

    def test_gait_invalid_types(self):
        for value in (None, 1.5, [], {}, (), 1j):
            with self.subTest(value=value):
                self.assert_validation(Commander._valid_gait, value, False)

    @unittest.expectedFailure
    def test_gait_integer_indices(self):
        # Known bug: .lower() is called on an integer before checking its range.
        for value in range(4):
            self.assert_validation(Commander._valid_gait, value, True)

    @unittest.expectedFailure
    def test_gait_out_of_range_indices(self):
        # Invalid integers should be rejected without raising AttributeError.
        for value in (-1, 4):
            self.assert_validation(Commander._valid_gait, value, False)

    @unittest.expectedFailure
    def test_unknown_gait_names(self):
        # Known bug: unknown strings are compared to integer bounds.
        for value in ("", "run", " walk ", "0"):
            self.assert_validation(Commander._valid_gait, value, False)

    @unittest.expectedFailure
    def test_optional_absolute_height(self):
        # None means leave the component unchanged, including height.
        for value in ((None, None, None), (0, 0, None)):
            self.assert_validation(Commander._valid_translation, value, True, False)


if __name__ == "__main__":
    unittest.main()
