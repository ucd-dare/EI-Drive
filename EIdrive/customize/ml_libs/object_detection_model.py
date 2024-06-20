"""
Since vehicle typically use the same ML model, the class has vehicles share the same model to avoid repeated memory waste.
"""

import cv2
import torch
import numpy as np
from collections import deque


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
        # device = torch.device('cuda')
        # print('Device:', device)

        # YOLOv5 model
        self.object_detector_yolo = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        self.object_detector_yolo.cuda()

        # SSD model
        self.utils = torch.hub.load('NVIDIA/DeepLearningExamples:torchhub', 'nvidia_ssd_processing_utils')
        self.object_detector_SSD = torch.hub.load('NVIDIA/DeepLearningExamples:torchhub', 'nvidia_ssd', trust_repo=True)
        # self.object_detector.to('cuda')
        self.object_detector_SSD.eval()

    def visualize_yolo_bbx(self, yolo_result, original_image, detection_index):
        """
        Overlay bounding boxes detected by YOLO on the given image.

        Parameters
        ----------
        yolo_result : yolo.Result
            Detection outcomes from YoloV5.
        original_image : np.ndarray
            The original image to which detections will be overlaid.
        detection_index : int
            Specific index for detection.

        Returns
        ----------
        modified_image : np.ndarray
            Image with the overlaid bounding boxes.
        """

        bounding_data = yolo_result.xyxy[detection_index]
        bounding_data = bounding_data.cpu().detach().numpy() if bounding_data.is_cuda else bounding_data.detach().numpy()

        for entry in bounding_data:
            label = int(entry[5])
            label_name = yolo_result.names[label]

            if is_vehicle_in_cococlass(label):
                label_name = 'vehicle'

            x1, y1, x2, y2 = map(int, entry[:4])
            cv2.rectangle(original_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(original_image, label_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 1)

        return original_image

    def visualize_ssd_bbx(self, best_results_per_input, classes_to_labels, image, index):
        """
        Overlay bounding boxes detected by SSD on the given image.

        Parameters
        ----------
        best_results_per_input : list of tuple
            Contains detection results for each image input, where each tuple comprises bounding boxes, classes, and confidences.

        classes_to_labels : dict
            A mapping from class IDs to their corresponding string labels.

        image : np.ndarray
            The original image on which the bounding boxes will be overlaid.

        index : int
            Specific index for detection (currently unused in the function).

        Returns
        ----------
        imagechange : np.ndarray
            The input image with overlaid bounding boxes and associated labels.

        Notes
        -----
        The function assumes that the image dimensions are fixed at 800x600 pixels for calculating bounding box coordinates.
        """

        imagechange = image.copy()  # Create a copy of the input image

        for image_idx in range(len(best_results_per_input)):
            bboxes, classes, confidences = best_results_per_input[image_idx]
            for idx in range(len(bboxes)):
                left, bot, right, top = bboxes[idx]
                x, w = [int(val * 800) for val in [left, right - left]]
                y, h = [int(val * 600) for val in [bot, top - bot]]
                cv2.rectangle(imagechange, (x, y), (x + w, y + h), (0, 255, 0), 2)

                label = classes_to_labels[classes[idx] - 1]
                confidence = int(confidences[idx] * 100)
                text = f"{label} {confidence}%"
                cv2.putText(imagechange, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 1)

        return imagechange


def is_vehicle_in_cococlass(label):
    """
    Determine if the provided label corresponds to a vehicle class in the COCO dataset.

    Parameters
    ----------
    label : int
        Predicted class label from YOLO detection.

    Returns
    -------
    bool
        True if the label corresponds to a vehicle class, otherwise False.
    """
    vehicle_classes = {1, 2, 3, 5, 7}
    return label in vehicle_classes
