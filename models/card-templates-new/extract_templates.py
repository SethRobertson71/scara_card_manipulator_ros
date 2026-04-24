#!/usr/bin/env python3
"""Extract individual Skip-Bo cards and number templates from all_cards.jpg."""

from __future__ import annotations

import os
from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass
class CardDet:
    x: int
    y: int
    w: int
    h: int
    cx: float
    cy: float


def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def find_cards(img: np.ndarray) -> list[CardDet]:
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Colored card face mask (drops white background).
    sat_mask = cv2.inRange(hsv[:, :, 1], 35, 255)
    val_mask = cv2.inRange(hsv[:, :, 2], 40, 255)
    mask = cv2.bitwise_and(sat_mask, val_mask)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    h_img, w_img = img.shape[:2]
    out: list[CardDet] = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < 15000:
            continue
        x, y, w, h = cv2.boundingRect(c)
        aspect = w / float(h)
        if not (0.35 <= aspect <= 0.95):
            continue

        # Expand bounds to include white border around colored face.
        pad_x = int(0.10 * w)
        pad_y = int(0.08 * h)
        x0 = max(0, x - pad_x)
        y0 = max(0, y - pad_y)
        x1 = min(w_img, x + w + pad_x)
        y1 = min(h_img, y + h + pad_y)
        ww = x1 - x0
        hh = y1 - y0
        out.append(CardDet(x0, y0, ww, hh, x0 + ww / 2.0, y0 + hh / 2.0))

    # Remove near-duplicates by IoU.
    deduped: list[CardDet] = []
    for cand in sorted(out, key=lambda d: d.w * d.h, reverse=True):
        keep = True
        for ex in deduped:
            iou = bbox_iou((cand.x, cand.y, cand.w, cand.h), (ex.x, ex.y, ex.w, ex.h))
            if iou > 0.45:
                keep = False
                break
        if keep:
            deduped.append(cand)

    return deduped


def bbox_iou(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> float:
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ax2, ay2 = ax + aw, ay + ah
    bx2, by2 = bx + bw, by + bh

    ix1 = max(ax, bx)
    iy1 = max(ay, by)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    iw = max(0, ix2 - ix1)
    ih = max(0, iy2 - iy1)
    inter = iw * ih
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


def cluster_rows(cards: list[CardDet], y_thresh: float = 120.0) -> list[list[CardDet]]:
    rows: list[list[CardDet]] = []
    for c in sorted(cards, key=lambda d: d.cy):
        placed = False
        for row in rows:
            avg_y = sum(r.cy for r in row) / len(row)
            if abs(c.cy - avg_y) < y_thresh:
                row.append(c)
                placed = True
                break
        if not placed:
            rows.append([c])

    for row in rows:
        row.sort(key=lambda d: d.cx)
    rows.sort(key=lambda r: sum(d.cy for d in r) / len(r))
    return rows


def cluster_rows_exact(cards: list[CardDet], row_count: int) -> list[list[CardDet]]:
    """Cluster cards into a fixed number of rows based on y center."""
    ys = np.array([c.cy for c in cards], dtype=np.float32)
    sort_idx = np.argsort(ys)
    groups = np.array_split(sort_idx, row_count)

    rows: list[list[CardDet]] = []
    for g in groups:
        row = [cards[int(i)] for i in g]
        row.sort(key=lambda d: d.cx)
        rows.append(row)
    rows.sort(key=lambda r: sum(d.cy for d in r) / len(r))
    return rows


def normalize_card_orientation(card_bgr: np.ndarray) -> np.ndarray:
    """Rotate and crop card to a consistent portrait view."""
    hsv = cv2.cvtColor(card_bgr, cv2.COLOR_BGR2HSV)
    sat_mask = cv2.inRange(hsv[:, :, 1], 35, 255)
    val_mask = cv2.inRange(hsv[:, :, 2], 40, 255)
    mask = cv2.bitwise_and(sat_mask, val_mask)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    ys, xs = np.where(mask > 0)
    if len(xs) < 100:
        out = cv2.resize(card_bgr, (200, 300), interpolation=cv2.INTER_CUBIC)
        return out

    pts = np.column_stack((xs.astype(np.float32), ys.astype(np.float32)))
    mean, eigenvectors, _ = cv2.PCACompute2(pts, mean=None)
    vx, vy = eigenvectors[0]
    angle_deg = math.degrees(math.atan2(vy, vx))

    # Rotate major axis toward vertical (90 deg from x-axis).
    rotate_deg = 90.0 - angle_deg
    cx, cy = float(card_bgr.shape[1]) / 2.0, float(card_bgr.shape[0]) / 2.0
    rot_m = cv2.getRotationMatrix2D((cx, cy), rotate_deg, 1.0)
    rotated = cv2.warpAffine(card_bgr, rot_m, (card_bgr.shape[1], card_bgr.shape[0]), flags=cv2.INTER_CUBIC)

    hsv_r = cv2.cvtColor(rotated, cv2.COLOR_BGR2HSV)
    sat_mask_r = cv2.inRange(hsv_r[:, :, 1], 35, 255)
    val_mask_r = cv2.inRange(hsv_r[:, :, 2], 40, 255)
    mask_r = cv2.bitwise_and(sat_mask_r, val_mask_r)

    contours, _ = cv2.findContours(mask_r, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        out = cv2.resize(rotated, (200, 300), interpolation=cv2.INTER_CUBIC)
        return out

    c = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(c)
    pad_x = int(0.1 * w)
    pad_y = int(0.08 * h)
    x0 = max(0, x - pad_x)
    y0 = max(0, y - pad_y)
    x1 = min(rotated.shape[1], x + w + pad_x)
    y1 = min(rotated.shape[0], y + h + pad_y)
    crop = rotated[y0:y1, x0:x1]

    if crop.shape[1] > crop.shape[0]:
        crop = cv2.rotate(crop, cv2.ROTATE_90_CLOCKWISE)

    return cv2.resize(crop, (200, 300), interpolation=cv2.INTER_CUBIC)


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

    cards = find_cards(img)
    if len(cards) < 16:
        raise RuntimeError(f"Expected at least 16 cards, found {len(cards)}")

    rows = cluster_rows(cards)

    # Left 4x4 grid contains backs + 1..12. Right side has 3 wild cards.
    x_centers = np.array([c.cx for c in cards], dtype=np.float32)
    x_cut = float(np.percentile(x_centers, 75))
    left_grid = [c for c in cards if c.cx <= x_cut]
    if len(left_grid) < 16:
        # Fallback: take 16 left-most cards.
        left_grid = sorted(cards, key=lambda d: d.cx)[:16]

    left_rows = cluster_rows_exact(left_grid, row_count=4)

    # Save all card crops for inspection.
    idx = 1
    for row in rows:
        for c in row:
            crop = img[c.y:c.y + c.h, c.x:c.x + c.w]
            crop = normalize_card_orientation(crop)
            cv2.imwrite(os.path.join(cards_dir, f"card_{idx:02d}.png"), crop)
            idx += 1

    # Build 1..12 templates from known grid rows/columns.
    # Expected row structure (left grid):
    # row0: backs, row1: 1-4, row2: 5-8, row3: 9-12
    # There are extra wild cards on the far right in rows 0..2.
    mapping = {
        (1, 0): 1, (1, 1): 2, (1, 2): 3, (1, 3): 4,
        (2, 0): 5, (2, 1): 6, (2, 2): 7, (2, 3): 8,
        (3, 0): 9, (3, 1): 10, (3, 2): 11, (3, 3): 12,
    }

    written = []
    for (r, c), num in mapping.items():
        if r >= len(left_rows) or c >= len(left_rows[r]):
            continue

        det = left_rows[r][c]
        card = img[det.y:det.y + det.h, det.x:det.x + det.w]
        card = normalize_card_orientation(card)

        # Center-focused ROI for the big digit.
        x0, y0, w, h = int(0.22 * 200), int(0.18 * 300), int(0.56 * 200), int(0.64 * 300)
        roi = card[y0:y0 + h, x0:x0 + w]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        bin_img = cv2.adaptiveThreshold(
            blur,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            21,
            7,
        )

        out_path = os.path.join(templates_dir, f"{num}.png")
        cv2.imwrite(out_path, bin_img)
        written.append(num)

    # Also save an overlay of detections to help debugging.
    debug = img.copy()
    for row_id, row in enumerate(rows):
        for col_id, d in enumerate(row):
            cv2.rectangle(debug, (d.x, d.y), (d.x + d.w, d.y + d.h), (0, 255, 255), 2)
            cv2.putText(
                debug,
                f"r{row_id}c{col_id}",
                (d.x, max(20, d.y - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
    # Mark left-grid cards in green for mapping clarity.
    for row_id, row in enumerate(left_rows):
        for col_id, d in enumerate(row):
            cv2.rectangle(debug, (d.x, d.y), (d.x + d.w, d.y + d.h), (0, 255, 0), 2)
            cv2.putText(
                debug,
                f"L{row_id},{col_id}",
                (d.x, d.y + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

    cv2.imwrite(os.path.join(base, "all_cards_detected_overlay.jpg"), debug)

    print(f"Detected cards: {len(cards)}")
    print(f"Rows: {[len(r) for r in rows]}")
    print(f"Left-grid rows: {[len(r) for r in left_rows]}")
    print(f"Wrote card crops: {idx - 1} to {cards_dir}")
    print(f"Wrote number templates: {sorted(written)} to {templates_dir}")


if __name__ == "__main__":
    main()
