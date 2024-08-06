"""
The class contains the ML model and load them when initializing.
"""

import torch


class MLModel(object):
    """
    Class to load and share Machine Learning (ML) models. We make it an independent attribute in GameWorld to better manage ML
    algorithm implementation.

    Parameters
    ----------
    load_model : bool
        Whether apply ML models in the simulation. The ML model will be loaded when True. Please make sure
        relative lib and package has been installed before loading.

    Attributes
    ----------
    object_detection_model : EIdrive object.
        The machine learning manager class.
    """

    def __init__(self, load_model=False):
        
        self.object_detection_model = None

        if load_model:
            # Load the ML models into memory
            self.object_detection_model = ObjectDetectionModel()

class ObjectDetectionModel(object):
    """
    Manager for object detection machine learning models.

    Attributes:
    ----------
    detector_yolo : torch model
        YoloV5 model from PyTorch hub.

    utils : torch utilities
        Processing utilities for SSD model.

    detector_SSD : torch model
        NVIDIA SSD model.
    """

    def __init__(self):

        # YOLOv5 model
        self.object_detector_yolo = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        self.object_detector_yolo.cuda()

        # SSD model
        self.utils = torch.hub.load('NVIDIA/DeepLearningExamples:torchhub', 'nvidia_ssd_processing_utils')
        self.object_detector_SSD = torch.hub.load('NVIDIA/DeepLearningExamples:torchhub', 'nvidia_ssd', trust_repo=True)
        self.object_detector_SSD.to('cuda')
        self.object_detector_SSD.eval()
