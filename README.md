# PylonProxy
small wrapper class to expose Basler's pylon library to Matlab (or C++)
it grabs the first GigE camera it finds and creates a ring buffer with the images from a continuous stream.
from Matlab you can pull an image, process it, and return the spot in the ring buffer for new images.

works with the CamView (C++) and mCamView (Matlab) tools found in the other repositories
