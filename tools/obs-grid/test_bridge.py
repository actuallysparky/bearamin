import unittest
from unittest.mock import patch
import bridge

class Tests(unittest.TestCase):
    def test_codec_matches_protocol_unicode_and_bounds(self):
        value = {"op": "identity", "data": {"name": "熊"}}
        self.assertEqual(bridge.decode(bridge.encode(value)), value)
        self.assertIsNone(bridge.decode([0, 1, 2]))
        self.assertIsNone(bridge.decode(bridge.PREFIX + [0, 255]))
        with self.assertRaises(ValueError):
            bridge.encode({"large": "x" * 5000})
    def test_frame_bounds_and_stale_data(self):
        state = bridge.State()
        with patch("bridge.time.monotonic", return_value=10):
            state.put({"mm": [120] * 64})
            self.assertEqual(state.get()["mm"], [120] * 64)
        with patch("bridge.time.monotonic", return_value=12):
            self.assertEqual(state.get()["mm"], [None] * 64)
        for bad in ([True]*64, [float("nan")]*64, [4001]*64, [0]*64, [1]*63):
            self.assertIsNone(bridge.valid_frame({"mm": bad}))
        self.assertIsNotNone(bridge.valid_frame({"mm": [None] * 64}))

if __name__ == "__main__":
    unittest.main()
