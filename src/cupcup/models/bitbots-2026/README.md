# Cupcup finals model

This directory is the package-local copy used by the finals build. It is not
loaded from the initial-round workspace.

The model was previously validated by the team in the initial-round project
as an OpenCV-compatible YOEO detection-head export. Its upstream provenance
and SHA-256 are recorded in `provenance.json`. Before public redistribution,
the team must confirm the upstream weight licence; the executable also keeps a
traditional OpenCV detector fallback for development without the model.

## Upstream review, 2026-10-08

The source ONNX URL responds with HTTP 200; its 28,358,260-byte original is
not the 25,832,307-byte package-local converted export. This review checked
headers and the model configuration, not the original artifact's hash.

The upstream [model configuration](https://data.bit-bots.de/models/2026_07_01_yoeo_x_wm/model_config.yaml)
lists detection classes `ball` and `robot`, and segmentation classes
`background` and `lines`. It supplies no player number or team class; those
must not be inferred from the robot class alone. The local export/decoder
uses the detection head, not a field-line localization system.

The [YOEO code repository licence](https://github.com/bit-bots/YOEO/blob/main/LICENSE)
contains GPL-3.0 text. The model's directory and ONNX subdirectory contained
no separate licence file on this review date. Neither code licence nor a
working public download establishes the exact weight's redistribution terms.
`weight_license_confirmed` therefore remains false; do not label this weight
MIT or release it as licence-cleared. Final submission/release still requires
the team's confirmation of the weight terms and competition compliance.
