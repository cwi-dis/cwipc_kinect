import unittest
import cwipc
import _cwipc_kinect
import os
import sys
import tempfile

#
# Find directories for test inputs and outputs
#
_thisdir=os.path.dirname(os.path.join(os.getcwd(), __file__))
_topdir=os.path.dirname(_thisdir)
TEST_FIXTURES_DIR=os.path.join(_topdir, "tests", "fixtures")
TEST_FIXTURES_PLAYBACK_CONFIG=os.path.join(TEST_FIXTURES_DIR, "input", "kinect_recording", "cameraconfig.json")

class TestApi(unittest.TestCase):
    
    def _open_grabber(self):
        try:
            grabber = _cwipc_kinect.cwipc_kinect("auto")
        except cwipc.CwipcError as arg:
            if str(arg) == 'cwipc_kinect: no kinect cameras found':
                self.skipTest(str(arg))
            raise
        return grabber
        
    def test_cwipc_kinect(self):
        """Test that we can grab a kinect image"""
        grabber = None
        pc = None
        try:
            grabber = self._open_grabber()
            self.assertFalse(grabber.eof())
            self.assertTrue(grabber.available(True))
            pc = grabber.get()
            self.assertIsNotNone(pc)
            assert pc # Only to keep linters happy
            # It seems the first pointcloud could be empty. Unsure why...
            if pc.count() == 0:
                pc = grabber.get()
                self.assertIsNotNone(pc)
                assert pc # Only to keep linters happy
            self._verify_pointcloud(pc)
        finally:
            if grabber: grabber.free()
            if pc: pc.free()

    def test_cwipc_kinect_tileinfo(self):
        """Test that we can get tileinfo from a kinect grabber"""
        grabber = None
        try:
            grabber = self._open_grabber()
            nTile = grabber.maxtile()
            self.assertGreaterEqual(nTile, 1)
            # Assure the non-tiled-tile exists and points nowhere.
            tileInfo = grabber.get_tileinfo_dict(0)
            self.assertIsNotNone(tileInfo)
            assert tileInfo # Only to keep linters happy
            self.assertIn('normal', tileInfo)
            self.assertIn('cameraName', tileInfo)
            self.assertIn('cameraMask', tileInfo)
            self.assertIn('ncamera', tileInfo)
            # Untrue if multiple realsenses connected: self.assertLessEqual(tileInfo['ncamera'], 1)
            # Test some minimal conditions for other tiles
            for i in range(1, nTile):
                tileInfo = grabber.get_tileinfo_dict(i)
                self.assertIsNotNone(tileInfo)
                assert tileInfo # Only to keep linters happy
                if i in (1, 2, 4, 8, 16, 32, 64, 128):
                    # These tiles should exist and have a normal and camera ID (which may be None)
                    self.assertIn('normal', tileInfo)
                    self.assertIn('cameraName', tileInfo)
        finally:
            if grabber: grabber.free()

    @unittest.skipIf(sys.platform=='linux' and not 'DISPLAY' in os.environ, "Test requires X server/OpenGL")
    def test_cwipc_k4aplayback(self):
        """Test that we can grab a kinect image from the playback grabber"""
        grabber = None
        pc = None
        if not os.path.exists(TEST_FIXTURES_PLAYBACK_CONFIG):
            self.skipTest(f'Playback config file {TEST_FIXTURES_PLAYBACK_CONFIG} not found')
        try:
            grabber = _cwipc_kinect.cwipc_k4aplayback(TEST_FIXTURES_PLAYBACK_CONFIG)
            self.assertFalse(grabber.eof())
            self.assertTrue(grabber.available(True))
            pc = grabber.get()
            self.assertIsNotNone(pc)
            assert pc # Only to keep linters happy
            self._verify_pointcloud(pc)
        finally:
            if grabber: grabber.free()
            if pc: pc.free()
            
    @unittest.skipIf(sys.platform=='linux' and not 'DISPLAY' in os.environ, "Test requires X server/OpenGL")
    def test_cwipc_k4aplayback_seek(self):
        """Test that we can grab a kinect image from the playback grabber"""
        grabber = None
        pc = None
        if not os.path.exists(TEST_FIXTURES_PLAYBACK_CONFIG):
            self.skipTest(f'Playback config file {TEST_FIXTURES_PLAYBACK_CONFIG} not found')
        try:
            grabber = _cwipc_kinect.cwipc_k4aplayback(TEST_FIXTURES_PLAYBACK_CONFIG)
            self.assertFalse(grabber.eof())
            self.assertTrue(grabber.available(True))
            result = grabber.seek(1600233)
            self.assertTrue(result)
            result = grabber.seek(5000000)
            self.assertFalse(result)
            pc = grabber.get()
            self.assertIsNotNone(pc)
            assert pc # Only to keep linters happy
            self._verify_pointcloud(pc)
        finally:
            if grabber: grabber.free()
            if pc: pc.free()

    def _verify_pointcloud(self, pc : cwipc.cwipc_wrapper) -> None:
        points = pc.get_points()
        self.assertGreater(len(points), 1)
        halfway = int((len(points)+1)/2)
        p0 = points[0].x, points[0].y, points[0].z, points[0].r, points[0].g, points[0].b
        p1 = points[halfway].x, points[halfway].y, points[halfway].z, points[halfway].r, points[halfway].g, points[halfway].b
        self.assertNotEqual(p0, p1)
   
if __name__ == '__main__':
    unittest.main()
