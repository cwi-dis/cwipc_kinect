import ctypes
import ctypes.util
import warnings
from cwipc.util import CwipcError, CWIPC_API_VERSION, cwipc_tiledsource
from cwipc.util import cwipc_tiledsource_p

__all__ = [
    "cwipc_kinect",
    "_cwipc_kinect_dll"
]

_cwipc_kinect_dll_reference = None

#
# NOTE: the signatures here must match those in cwipc_util/api.h or all hell will break loose
#
def _cwipc_kinect_dll(libname=None):
    """Load the cwipc_util DLL and assign the signatures (if not already loaded)"""
    global _cwipc_kinect_dll_reference
    if _cwipc_kinect_dll_reference: return _cwipc_kinect_dll_reference
    
    if libname == None:
        libname = ctypes.util.find_library('cwipc_kinect')
        if not libname:
            raise RuntimeError('Dynamic library cwipc_kinect not found')
    assert libname
    _cwipc_kinect_dll_reference = ctypes.CDLL(libname)
    
    _cwipc_kinect_dll_reference.cwipc_kinect.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_char_p), ctypes.c_ulong]
    _cwipc_kinect_dll_reference.cwipc_kinect.restype = cwipc_tiledsource_p

    return _cwipc_kinect_dll_reference
        
def cwipc_kinect(conffile=None):
    """Returns a cwipc_source object that grabs from a kinect camera and returns cwipc object on every get() call."""
    errorString = ctypes.c_char_p()
    if conffile:
        conffile = conffile.encode('utf8')
    else:
        conffile = None
    rv = _cwipc_kinect_dll().cwipc_kinect(conffile, ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString and not rv:
        raise CwipcError(errorString.value.decode('utf8'))
    if errorString:
        warnings.warn(errorString.value.decode('utf8'))
    if rv:
        return cwipc_tiledsource(rv)
    return None

def cwipc_k4aoffline(conffile=None):
    """Returns a cwipc_source object that grabs from kinect camera recordings and returns cwipc objects on every get() call."""
    errorString = ctypes.c_char_p()
    if conffile:
        conffile = conffile.encode('utf8')
    else:
        conffile = None
    rv = _cwipc_kinect_dll().cwipc_k4aoffline(conffile, ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString and not rv:
        raise CwipcError(errorString.value.decode('utf8'))
    if errorString:
        warnings.warn(errorString.value.decode('utf8'))
    if rv:
        return cwipc_tiledsource(rv)
    return None
         
def main():
    grabber = cwipc_kinect()
    pc = grabber.get()
    if not pc:
        print('Could not read pointcloud from kinect grabber')
    points = pc.get_points()
    print('Pointcloud contained %d points' % len(points))
    
if __name__ == '__main__':
    main()
    
    
