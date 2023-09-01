# -*- coding: utf-8 -*-

"""
Since multiple CAV normally use the same ML/DL model,
here we have this class to enable different CAVs share the same model to
 avoid duplicate memory consumption.
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib


import cv2
import torch
import numpy as np
# from torchvision.models.detection import ssdlite320_mobilenet_v3_large
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class MLManager(object):
    """
    A class that should contain all the ML models you want to initialize.

    Attributes
    -object_detector : torch_detector
        The YoloV5 detector load from pytorch.

    """

    def __init__(self):
        # device = torch.device('cuda')
        # print('Device:', device)

        self.object_detector_yolo = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        self.object_detector_yolo.cuda()

        self.utils = torch.hub.load('NVIDIA/DeepLearningExamples:torchhub', 'nvidia_ssd_processing_utils')
        self.object_detector_SSD = torch.hub.load('NVIDIA/DeepLearningExamples:torchhub', 'nvidia_ssd', trust_repo=True)
        # self.object_detector.to('cuda')
        self.object_detector_SSD.eval()

    def draw_2d_box(self, result, rgb_image, index):
        """
        Draw 2d bounding box based on the yolo detection.

        Args:
            -result (yolo.Result):Detection result from yolo 5.
            -rgb_image (np.ndarray): Camera rgb image.
            -index(int): Indicate the index.

        Returns:
            -rgb_image (np.ndarray): camera image with bbx drawn.
        """
        # torch.Tensor
        bounding_box = result.xyxy[index]
        if bounding_box.is_cuda:
            bounding_box = bounding_box.cpu().detach().numpy()
        else:
            bounding_box = bounding_box.detach().numpy()

        for i in range(bounding_box.shape[0]):
            detection = bounding_box[i]

            # the label has 80 classes, which is the same as coco dataset
            label = int(detection[5])
            label_name = result.names[label]

            if is_vehicle_cococlass(label):
                label_name = 'vehicle'

            x1, y1, x2, y2 = int(
                detection[0]), int(
                detection[1]), int(
                detection[2]), int(
                detection[3])
            cv2.rectangle(rgb_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # draw text on it
            cv2.putText(rgb_image, label_name, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 1)

        return rgb_image

    def draw_2d_box_SSD(self, best_results_per_input, classes_to_labels, image, index):
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


def is_vehicle_cococlass(label):
    """
    Check whether the label belongs to the vehicle class according
    to coco dataset.
    Args:
        -label(int): yolo detection prediction.
    Returns:
        -is_vehicle: bool
            whether this label belongs to the vehicle class
    """
    vehicle_class_array = np.array([1, 2, 3, 5, 7], dtype=np.int)
    return True if 0 in (label - vehicle_class_array) else False
