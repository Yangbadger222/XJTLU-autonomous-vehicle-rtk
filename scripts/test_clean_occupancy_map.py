import tempfile
import unittest
from pathlib import Path

from clean_occupancy_map import clean_components, read_pgm, write_pgm


class CleanOccupancyMapTest(unittest.TestCase):
    def test_removes_small_components_and_preserves_larger_obstacle(self):
        width, height = 8, 5
        pixels = bytearray([254] * (width * height))
        pixels[1 * width + 1] = 0
        pixels[1 * width + 5] = 0
        pixels[2 * width + 5] = 0
        for row, col in ((3, 1), (3, 2), (4, 1), (4, 2)):
            pixels[row * width + col] = 0

        components, cells = clean_components(width, height, pixels, 50, 3)

        self.assertEqual((components, cells), (2, 3))
        self.assertEqual(pixels[1 * width + 1], 254)
        self.assertEqual(pixels[1 * width + 5], 254)
        self.assertEqual(pixels[3 * width + 1], 0)

    def test_unknown_boundary_is_not_changed_to_free(self):
        width, height = 3, 3
        pixels = bytearray([205] * 9)
        pixels[4] = 0

        clean_components(width, height, pixels, 50, 3)

        self.assertEqual(pixels[4], 205)

    def test_binary_pgm_round_trip(self):
        pixels = bytearray([0, 205, 254, 254])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "map.pgm"
            write_pgm(path, 2, 2, 255, pixels)
            self.assertEqual(read_pgm(path), (2, 2, 255, pixels))


if __name__ == "__main__":
    unittest.main()
