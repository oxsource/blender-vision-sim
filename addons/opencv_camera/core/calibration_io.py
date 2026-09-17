"""Calibration file import/export.

Supported inputs (auto-detected from the content, not the extension):

* OpenCV ``FileStorage`` YAML/XML-ish YAML produced by ``cv2.calibrateCamera``
  (``camera_matrix`` / ``distortion_coefficients`` / ``image_width`` / ...).
* JSON with the same key layout (row-major ``camera_matrix`` list or nested).
* ROS ``camera_info`` YAML (``K``/``D``/``P`` are not used; the nested
  ``camera_matrix`` / ``distortion_coefficients`` blocks are).
* Kalibr ``camchain``/``cam`` YAML (``intrinsics: [fx, fy, cx, cy]``,
  ``distortion_coeffs``, ``distortion_model``).

Outputs: JSON and a ROS-style YAML mapping (readable by ``cv2.FileStorage`` and
by any YAML parser).

YAML is parsed with a dependency-free subset parser (Blender does not bundle
PyYAML): nested mappings by indentation, inline flow sequences, scalars.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

from .camera_model import (
    MODEL_BROWN_CONRADY,
    MODEL_FISHEYE,
    MODEL_RATIONAL,
    Distortion,
    Intrinsics,
)

__all__ = ["Calibration", "load_calibration", "save_calibration", "parse_yaml_subset"]


@dataclass
class Calibration:
    """A camera calibration: intrinsics, distortion and optional rectified K."""

    intrinsics: Intrinsics
    distortion: Distortion
    rectify_intrinsics: Optional[Intrinsics] = None
    source: str = ""
    model_name: str = ""

    @property
    def width(self) -> int:
        return self.intrinsics.width

    @property
    def height(self) -> int:
        return self.intrinsics.height


# ---------------------------------------------------------------------------
# minimal YAML subset parser
# ---------------------------------------------------------------------------
def _strip_comment(line: str) -> str:
    out = []
    quote = ""
    for ch in line:
        if quote:
            out.append(ch)
            if ch == quote:
                quote = ""
            continue
        if ch in "\"'":
            quote = ch
            out.append(ch)
            continue
        if ch == "#" and out and out[-1] in " \t":
            break
        out.append(ch)
    return "".join(out).rstrip()


def _split_flow(text: str) -> List[str]:
    """Split a top-level flow sequence body on commas."""
    items, buf, depth, quote = [], [], 0, ""
    for ch in text:
        if quote:
            buf.append(ch)
            if ch == quote:
                quote = ""
            continue
        if ch in "\"'":
            quote = ch
            buf.append(ch)
        elif ch in "[{":
            depth += 1
            if depth > 1:
                buf.append(ch)
        elif ch in "]}":
            depth -= 1
            if depth > 0:
                buf.append(ch)
        elif ch == "," and depth == 0:
            items.append("".join(buf).strip())
            buf = []
        else:
            buf.append(ch)
    if buf:
        items.append("".join(buf).strip())
    return [i for i in items if i]


def _parse_scalar(text: str):
    text = text.strip()
    if len(text) >= 2 and text[0] == text[-1] and text[0] in "\"'":
        return text[1:-1]
    if text.startswith("[") and text.endswith("]"):
        return [_parse_scalar(i) for i in _split_flow(text[1:-1])]
    if text.lower() in ("true", "false"):
        return text.lower() == "true"
    if text.lower() in ("null", "none", "~"):
        return None
    try:
        return int(text)
    except ValueError:
        pass
    try:
        return float(text)
    except ValueError:
        return text


def parse_yaml_subset(text: str) -> Dict:
    """Parse the YAML subset used by calibration files into nested dicts/lists."""
    # normalise: drop directives, document markers and OpenCV type tags
    lines: List[Tuple[int, str]] = []
    for raw in text.splitlines():
        raw = _strip_comment(raw)
        if not raw.strip():
            continue
        if raw.lstrip().startswith(("%", "---", "...")):
            continue
        raw = raw.replace("!!opencv-matrix", "").replace("!!python/object", "")
        indent = len(raw) - len(raw.lstrip(" "))
        lines.append((indent, raw.strip()))

    def parse_block(pos: int, indent: int) -> Tuple[Dict, int]:
        result: Dict = {}
        while pos < len(lines):
            level, content = lines[pos]
            if level < indent:
                break
            if level > indent:  # belongs to a parent (shouldn't happen)
                pos += 1
                continue
            key, sep, rest = content.partition(":")
            if not sep:
                raise ValueError(f"cannot parse YAML line: {content!r}")
            key = key.strip().strip("\"'")
            rest = rest.strip()
            if rest:
                result[key] = _parse_scalar(rest)
                pos += 1
                continue
            # nested block (mapping or multi-line flow sequence)
            nxt = pos + 1
            while nxt < len(lines) and lines[nxt][0] <= level:
                nxt += 1
            if nxt >= len(lines):
                result[key] = None
                pos += 1
                continue
            child_indent = lines[nxt][0]
            if lines[nxt][1].startswith("["):
                buffer = ""
                pos = nxt
                while pos < len(lines):
                    buffer += lines[pos][1]
                    if buffer.count("[") <= buffer.count("]"):
                        break
                    pos += 1
                result[key] = _parse_scalar(buffer)
                pos += 1
            else:
                child, pos = parse_block(nxt, child_indent)
                result[key] = child
        return result, pos

    doc, _ = parse_block(0, lines[0][0] if lines else 0)
    return doc


# ---------------------------------------------------------------------------
# extraction
# ---------------------------------------------------------------------------
def _floats(values) -> List[float]:
    if values is None:
        return []
    if isinstance(values, dict):
        values = values.get("data", [])
    if isinstance(values, (int, float)):
        return [float(values)]
    return [float(v) for v in values]


def _matrix(doc: Dict, key: str) -> List[float]:
    node = doc.get(key)
    if isinstance(node, dict):
        return _floats(node.get("data"))
    return _floats(node)


def _model_from_name(name: str) -> str:
    name = (name or "").strip().lower()
    if name in ("fisheye", "equidistant", "kannala_brandt", "kannala-brandt"):
        return MODEL_FISHEYE
    if name in ("rational", "rational_polynomial", "rational_polynomial_model"):
        return MODEL_RATIONAL
    return MODEL_BROWN_CONRADY


def _calibration_from_doc(doc: Dict, source: str) -> Calibration:
    width = int(doc.get("image_width") or doc.get("width") or 0)
    height = int(doc.get("image_height") or doc.get("height") or 0)
    resolution = doc.get("resolution")
    if isinstance(resolution, (list, tuple)) and len(resolution) >= 2:
        width, height = int(resolution[0]), int(resolution[1])

    matrix = _matrix(doc, "camera_matrix") or _matrix(doc, "K")
    intrinsics = _matrix(doc, "intrinsics")
    if len(matrix) >= 9:
        fx, fy = matrix[0], matrix[4]
        cx, cy = matrix[2], matrix[5]
    elif len(intrinsics) >= 4:  # Kalibr
        fx, fy, cx, cy = intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3]
    elif len(matrix) == 4:
        fx, fy, cx, cy = matrix
    else:
        raise ValueError(
            "no usable intrinsics found (expected 'camera_matrix' with 9 values, "
            "'K', or Kalibr 'intrinsics' with 4 values)"
        )
    if not width or not height:
        raise ValueError("calibration has no image size (image_width/image_height)")

    coeffs = _matrix(doc, "distortion_coefficients") or _matrix(doc, "distortion_coeffs")
    model = _model_from_name(
        doc.get("distortion_model") or doc.get("camera_model") or doc.get("model") or ""
    )
    distortion = Distortion.from_coefficients(coeffs, model=model)

    rectify = None
    projection = _matrix(doc, "projection_matrix") or _matrix(doc, "P")
    if len(projection) >= 12 and (projection[0] or projection[5]):
        rectify = Intrinsics(
            fx=projection[0], fy=projection[5], cx=projection[2], cy=projection[6],
            width=width, height=height,
        )

    return Calibration(
        intrinsics=Intrinsics(fx=fx, fy=fy, cx=cx, cy=cy, width=width, height=height),
        distortion=distortion,
        rectify_intrinsics=rectify,
        source=source,
        model_name=str(doc.get("distortion_model") or doc.get("camera_model") or ""),
    )


def load_calibration(path: str) -> Calibration:
    """Load a calibration file, auto-detecting JSON vs the YAML subset."""
    text = open(path, "r", encoding="utf-8", errors="replace").read()
    stripped = text.lstrip()
    if stripped.startswith("{"):
        doc = json.loads(text)
    else:
        doc = parse_yaml_subset(text)
    if not isinstance(doc, dict):
        raise ValueError(f"{os.path.basename(path)}: expected a mapping at the top level")
    # a Kalibr camchain nests the camera under 'cam0'/'cam1'/... or 'camera'
    for key in ("camera", "cam0", "cam"):
        nested = doc.get(key)
        if isinstance(nested, dict) and (
            nested.get("camera_matrix") or nested.get("intrinsics") or nested.get("K")
        ):
            doc = nested
            break
    return _calibration_from_doc(doc, source=path)


# ---------------------------------------------------------------------------
# export
# ---------------------------------------------------------------------------
def _matrix_block(indent: str, values: Sequence[float], rows: int, cols: int) -> str:
    body = ", ".join(repr(float(v)) for v in values)
    return (
        f"{indent}rows: {rows}\n"
        f"{indent}cols: {cols}\n"
        f"{indent}dt: d\n"
        f"{indent}data: [{body}]\n"
    )


def save_calibration(path: str, calib: Calibration) -> str:
    """Write a calibration file; ``.json`` writes JSON, anything else YAML."""
    intrinsics = calib.intrinsics.resolved()
    matrix = [
        intrinsics.fx, 0.0, intrinsics.cx,
        0.0, intrinsics.fy, intrinsics.cy,
        0.0, 0.0, 1.0,
    ]
    if calib.distortion.model == MODEL_FISHEYE:
        coeffs = calib.distortion.coefficients(4)
    elif calib.distortion.has_rational or calib.distortion.model == MODEL_RATIONAL:
        coeffs = calib.distortion.coefficients(8)
    else:
        coeffs = calib.distortion.coefficients(5)
    model = calib.distortion.model
    if model == MODEL_BROWN_CONRADY:
        model = "plumb_bob"
    elif model == MODEL_RATIONAL:
        model = "rational_polynomial"
    elif model == MODEL_FISHEYE:
        model = "equidistant"

    if path.lower().endswith(".json"):
        payload = {
            "image_width": intrinsics.width,
            "image_height": intrinsics.height,
            "camera_matrix": matrix,
            "distortion_model": model,
            "distortion_coefficients": coeffs,
        }
        if calib.rectify_intrinsics is not None:
            r = calib.rectify_intrinsics.resolved()
            payload["projection_matrix"] = [
                r.fx, 0.0, r.cx, 0.0,
                0.0, r.fy, r.cy, 0.0,
                0.0, 0.0, 1.0, 0.0,
            ]
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)
            handle.write("\n")
        return path

    lines = [
        f"image_width: {intrinsics.width}",
        f"image_height: {intrinsics.height}",
        "camera_matrix:",
        _matrix_block("  ", matrix, 3, 3).rstrip("\n"),
        f"distortion_model: {model}",
        "distortion_coefficients:",
        _matrix_block("  ", coeffs, 1, len(coeffs)).rstrip("\n"),
    ]
    if calib.rectify_intrinsics is not None:
        r = calib.rectify_intrinsics.resolved()
        lines += [
            "projection_matrix:",
            _matrix_block("  ", [r.fx, 0.0, r.cx, 0.0, 0.0, r.fy, r.cy, 0.0, 0.0, 0.0, 1.0, 0.0], 3, 4).rstrip("\n"),
        ]
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")
    return path