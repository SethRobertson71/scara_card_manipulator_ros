#!/usr/bin/env python3
"""Extract 12 individual cards and templates from all_cards.jpg."""

from __future__ import annotations

import os
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class CardDet:
    box: np.ndarray  # 4x2 float32
    cx: float
    cy: float
    w: float
    h: float


def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def order_points(pts: np.ndarray) -> np.ndarray:
    rect = np.zeros((4, 2), dtype=np.float32)
    s = pts.sum(axis=1)
    d = np.diff(pts, axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    rect[1] = pts[np.argmin(d)]
    rect[3] = pts[np.argmax(d)]
    return rect


def find_12_cards(img: np.ndarray) -> list[CardDet]:
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # White cards on dark table.
    mask = cv2.inRange(gray, 145, 255)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    h, w = img.shape[:2]
    img_area = float(h * w)

    dets: list[CardDet] = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < 0.01 * img_area or area > 0.20 * img_area:
            continue

        rect = cv2.minAreaRect(c)
        (_, _), (rw, rh), _ = rect
        if rw < 10 or rh < 10:
            continue
        short_side = min(rw, rh)
        long_side = max(rw, rh)
        aspect = short_side / long_side
        if not (0.55 <= aspect <= 0.80):
            continue

        box = cv2.boxPoints(rect).astype(np.float32)
        cx = float(np.mean(box[:, 0]))
        cy = float(np.mean(box[:, 1]))
        card_w = float(min(rw, rh))
        card_h = float(max(rw, rh))
        dets.append(CardDet(box=box, cx=cx, cy=cy, w=card_w, h=card_h))

    # Keep 12 largest by polygon area when extras appear.
    dets.sort(key=lambda d: cv2.contourArea(d.box.astype(np.float32)), reverse=True)
    dets = dets[:12]

    if len(dets) != 12:
        raise RuntimeError(f"Expected 12 cards, found {len(dets)}")

    # Force all detections to consistent axis-aligned rectangles to reduce skew.
    med_w = float(np.median([d.w for d in dets]))
    med_h = float(np.median([d.h for d in dets]))
    for d in dets:
        d.box = make_axis_aligned_box(d.cx, d.cy, med_w, med_h, w, h)
        d.w = med_w
        d.h = med_h

    return dets


def make_axis_aligned_box(
    cx: float,
    cy: float,
    w: float,
    h: float,
    img_w: int,
    img_h: int,
) -> np.ndarray:
    x0 = max(0.0, cx - w / 2.0)
    y0 = max(0.0, cy - h / 2.0)
    x1 = min(float(img_w - 1), cx + w / 2.0)
    y1 = min(float(img_h - 1), cy + h / 2.0)
    return np.array(
        [[x0, y0], [x1, y0], [x1, y1], [x0, y1]],
        dtype=np.float32,
    )


def sort_row_major(cards: list[CardDet]) -> list[CardDet]:
    # Split into top/bottom rows by y center then sort each row by x.
    cards_sorted = sorted(cards, key=lambda c: c.cy)
    top = sorted(cards_sorted[:6], key=lambda c: c.cx)
    bottom = sorted(cards_sorted[6:], key=lambda c: c.cx)
    return top + bottom


def warp_card(img: np.ndarray, box: np.ndarray, out_w: int = 200, out_h: int = 300) -> np.ndarray:
    src = order_points(box.astype(np.float32))
    dst = np.array(
        [[0, 0], [out_w - 1, 0], [out_w - 1, out_h - 1], [0, out_h - 1]],
        dtype=np.float32,
    )
    m = cv2.getPerspectiveTransform(src, dst)
    card = cv2.warpPerspective(img, m, (out_w, out_h), flags=cv2.INTER_CUBIC)
    return card


def preprocess_template(card: np.ndarray) -> np.ndarray:
    # Wider center ROI helps keep both digits for 10/11/12.
    x0, y0 = int(0.10 * 200), int(0.14 * 300)
    ww, hh = int(0.80 * 200), int(0.72 * 300)
    roi = card[y0:y0 + hh, x0:x0 + ww]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    return cv2.adaptiveThreshold(
        blur,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        21,
        7,
    )


def main() -> None:
    base = os.path.dirname(os.path.abspath(__file__))
    src_path = os.path.join(base, "all_cards.jpg")
    cards_dir = os.path.join(base, "cards")
    templates_dir = os.path.join(base, "templates")

    ensure_dir(cards_dir)
    ensure_dir(templates_dir)

    img = cv2.imread(src_path)
    if img is None:
        raise FileNotFoundError(src_path)

    cards = sort_row_major(find_12_cards(img))

    # Save perspective-rectified card crops and templates 1..12.
    for idx, det in enumerate(cards, start=1):
        card = warp_card(img, det.box, out_w=200, out_h=300)
        cv2.imwrite(os.path.join(cards_dir, f"card_{idx:02d}.png"), card)

        templ = preprocess_template(card)
        cv2.imwrite(os.path.join(templates_dir, f"{idx}.png"), templ)

    debug = img.copy()
    for idx, det in enumerate(cards, start=1):
        box_i = det.box.astype(np.int32)
        cv2.polylines(debug, [box_i], True, (0, 255, 255), 2)
        label_xy = tuple(np.min(box_i, axis=0))
        cv2.putText(
            debug,
            str(idx),
            (int(label_xy[0]), max(20, int(label_xy[1]) - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

    cv2.imwrite(os.path.join(base, "all_cards_detected_overlay.jpg"), debug)

    print("Detected cards: 12")
    print(f"Wrote card crops to {cards_dir}")
    print(f"Wrote templates 1..12 to {templates_dir}")


if __name__ == "__main__":
    main()
