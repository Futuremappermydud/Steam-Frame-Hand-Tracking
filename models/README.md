# Mercury hand tracking models

`fp32/` holds the two neural networks of Monado's **Mercury** hand tracker, unmodified:

| file | what | SHA-256 |
| --- | --- | --- |
| `grayscale_detection_160x160.onnx` | hand detector | `1f1a039a266e13dc186bb884430ebd9c8216bdda680ab08a533d4c671f27ed36` |
| `grayscale_keypoint_jan18.onnx` | 21-keypoint estimator | `40c0daa598cedb993b54fff17685231b7465d6db342656c401b01c2029efd1d5` |

## Credit

These models were trained and published by **Moshi Turner** at **Collabora, Ltd.** for the
[Monado](https://monado.freedesktop.org) OpenXR runtime. They are not our work.

- Source: <https://gitlab.freedesktop.org/monado/utilities/hand-tracking-models>, commit
  `37a8a81bc8f433ac6cbdf2471909d2bac74beca1` (the files are byte-identical to that commit).
- About Mercury: <https://monado.freedesktop.org/handtracking>
- Training code: <https://gitlab.freedesktop.org/monado/utilities/hand-tracking-playground/mercury_train>

## License

The upstream repository states no license for these files, and we found no license for them anywhere else
(checked November 2026). They are included here, with credit, only so the tracker can be installed without a
separate download. **They are not covered by this repository's MIT License**, and we claim no rights in them.

If you hold rights in these models and want them removed or labelled differently, open an issue and we will
do it.

## fp16

`fetch_models.py --fp16` checks the files above against their SHA-256 (downloading them from the source again
if one is missing or changed) and writes half-precision copies to `fp16/`, which the tracker uses by default.
`fp16/` is not stored in the repository: a source build generates it on the device, and the prebuilt release
includes the generated files (the same weights at half precision, with the same credit and no MIT License).
