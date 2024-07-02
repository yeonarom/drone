#!/usr/bin/env python

import torch
import torchvision
import torchreid

class ReidModel:
    def __init__(self, model_path):
        self.model = torchreid.models.build_model(name='resnet50', num_classes=751, pretrained=True)
        torchreid.utils.load_pretrained_weights(self.model, model_path)
        self.model.eval()

    def extract_features(self, image):
        transform = torchvision.transforms.Compose([
            torchvision.transforms.ToPILImage(),
            torchvision.transforms.Resize((128, 256)),
            torchvision.transforms.ToTensor(),
        ])
        image = transform(image).unsqueeze(0)
        with torch.no_grad():
            features = self.model(image)
        return features

    def compare_features(self, features1, features2):
        return torch.nn.functional.cosine_similarity(features1, features2).item()
    
class ObjectReidentifier:
    def __init__(self):
        self.reid_model = ReidModel('/home/yeonarom/catkin_ws/src/drone_operation/src/market.pth')
        self.object_features = {}
        self.next_id = 1

    def __call__(self, cv_image, tracked_objects):
        if tracked_objects is not None:
            reidentified_objects = self.reidentify_objects(cv_image, tracked_objects)
            return reidentified_objects

    def reidentify_objects(self, image, tracked_objects):
        reidentified_objects = []
        for obj in tracked_objects:
            x1, y1, x2, y2, track_id = obj
            cropped_image = image[int(y1):int(y2), int(x1):int(x2)]
            features = self.reid_model.extract_features(cropped_image)

            similarity = 0
            if track_id in self.object_features:
                similarity = self.reid_model.compare_features(self.object_features[track_id], features)

            if similarity > 0.7:
                reidentified_objects.append(track_id)
                self.object_features[track_id] = features
            else:
                best_match_id = None
                best_similarity = 0

                for existing_id, existing_features in self.object_features.items():
                    similarity = self.reid_model.compare_features(existing_features, features)
                    if similarity > best_similarity:
                        best_similarity = similarity
                        best_match_id = existing_id

                if best_similarity > 0.5:
                    reidentified_objects.append(best_match_id)
                    self.object_features[best_match_id] = features  # Update the features with the latest ones
                else:
                    new_id = self.next_id
                    self.next_id += 1
                    reidentified_objects.append(new_id)
                    self.object_features[new_id] = features

        return reidentified_objects