# Models compiled into `kestrel`

## yolox_nano.onnx — the demo's person detector

| | |
|---|---|
| What | YOLOX-nano, 416×416, COCO 80 classes (person = class 0) |
| From | Megvii-BaseDetection/YOLOX, release `0.1.1rc0`, `yolox_nano.onnx` |
| SHA-256 | `c789161ed43c8269fcd4e67c67eeeb4e80c622da2eb296a20bc6007bd18a0b7d` |
| Size | 3,659,407 bytes |
| Licence | **Apache-2.0** — [`LICENSE-YOLOX`](LICENSE-YOLOX), copyright Megvii Inc. |

`cmake/embed_file.cmake` turns it into a source file at build time, so the exe
needs no model beside it (`person_detector.hpp`). Apache-2.0 permits this in
closed-source software; what it asks is that the licence text and this notice
travel with a distributed binary — put `LICENSE-YOLOX` next to `kestrel.exe`
when you hand it to someone.

Chosen over Ultralytics YOLOv5/v8/11 because those are **AGPL-3.0**: compiled
in, they would put the whole of `kestrel` under the AGPL the moment the exe is
distributed.

Measured here (`test/person_detector_check`, `PERSON_TEST_IMAGE=`): on
OpenCV's grayscale `basketball1.png` sample (the look of the D435i's IR) it
finds both people, including one cut off by the frame edge, where HOG finds
none; ~10 ms per frame on a 4-thread desktop CPU, 28 ms on one thread.
