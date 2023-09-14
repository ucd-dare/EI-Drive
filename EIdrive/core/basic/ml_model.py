"""
The class contains the ML model and load them when initializing.
"""

import importlib


class MLModel(object):
    """
    Class to load and share ML models. We make it an independent attribute in GameWorld to better manage ML
    algorithm implementation.

    Parameters
    ----------
    apply_ml : bool
        Whether apply ML models in the simulation. The ML model will be loaded when True. Please make sure
        relative lib and package has been installed before loading.

    Attributes
    ----------
    object_detection_model : EIdrive object.
        The machine learning manager class.
    """

    def __init__(self, apply_ml=False):

        self.object_detection_model = None

        if apply_ml:
            # Import ML model
            object_detection_model = getattr(importlib.import_module(
                "EIdrive.customize.ml_libs.object_detection_model"), 'ObjectDetectionModel')

            # Load the ML models into memory
            self.object_detection_model = object_detection_model()



