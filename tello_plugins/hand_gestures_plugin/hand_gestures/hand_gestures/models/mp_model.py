from hand_gestures.models.template import Model
import mediapipe as mp
from mediapipe.tasks.python.vision import (
    GestureRecognizerOptions,
    GestureRecognizer,
)
from mediapipe.tasks.python import BaseOptions
from numpy._typing import NDArray
from typing import Optional
from mediapipe.tasks.python.vision import (
    GestureRecognizerResult,
)


class MediaPipeGesturesRecognizer(Model):
    def __init__(
        self,
        model_path: str,
        num_hands: int,
        min_det_conf: float,
        min_track_conf: float,
    ) -> None:
        self.model = GestureRecognizer.create_from_options(
            GestureRecognizerOptions(
                base_options=BaseOptions(model_asset_path=model_path),
                num_hands=num_hands,
                min_hand_detection_confidence=min_det_conf,
                min_tracking_confidence=min_track_conf,
            )
        )

    def predict(self, img: NDArray) -> GestureRecognizerResult:
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=img)
        return self.model.recognize(mp_image)
