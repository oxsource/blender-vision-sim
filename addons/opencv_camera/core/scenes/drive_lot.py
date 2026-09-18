"""Drive Scene car-park layout: bays, dividers, pillars, parked cars and labels.

Pure Python (no ``bpy``): the geometry the builder turns into meshes, and the
geometry the tests can check without Blender.  Markings, bay numbers and the
parked cars all read this one layout, so a number always sits in front of *its*
bay and no parked car ends up inside a column.

Everything is deterministic - the same settings always produce the same car park,
which is what makes a recorded clip reproducible from its ``clip.json``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

#: what each row's bays are called (side -> letter)
ROW_LETTERS: Dict[float, str] = {-1.0: "A", 1.0: "B"}

#: a column keeps this much of the row free, so no car is parked into it [m]
PILLAR_CLEARANCE = 1.7

#: how far a column's centre sits into the bay row (half a 0.6 m column plus a
#: hand's width off the aisle edge)
PILLAR_OFFSET = 0.45

#: parked vehicles, cycled over the occupied bays: length / width / height /
#: paint index (a bit of variety, but always the same variety)
PARKED_FORMS: Tuple[Tuple[float, float, float, int], ...] = (
    (4.45, 1.82, 1.52, 0),  # sedan
    (4.75, 1.92, 1.68, 1),  # SUV
    (4.20, 1.75, 1.45, 2),  # hatchback
    (4.95, 1.98, 1.92, 3),  # van
    (4.60, 1.86, 1.58, 4),  # estate
)

#: the paints of :data:`PARKED_FORMS` (the builder turns these into materials)
PARKED_COLORS: Tuple[Tuple[float, float, float, float], ...] = (
    (0.78, 0.78, 0.76, 1.0),   # white
    (0.62, 0.63, 0.65, 1.0),   # silver
    (0.10, 0.11, 0.12, 1.0),   # dark grey
    (0.12, 0.16, 0.32, 1.0),   # dark blue
    (0.34, 0.07, 0.08, 1.0),   # dark red
)


@dataclass(frozen=True)
class Bay:
    """One parking bay (a rect next to the aisle)."""

    side: float   #: -1 left row, +1 right row
    index: int    #: 0-based, counted from the -Y end
    label: str    #: "A01"
    x: float      #: centre [m]
    y: float      #: centre [m]
    depth: float  #: aisle edge -> wall [m]
    width: float  #: along the aisle [m]


@dataclass(frozen=True)
class ParkedCar:
    """One parked vehicle, standing in its bay nose-in."""

    bay: Bay
    length: float
    width: float
    height: float
    paint: int     #: index into :data:`PARKED_COLORS`
    yaw: float     #: degrees; the nose points at the wall


def bay_count(lot_length: float, bay_width: float) -> int:
    """How many bays fit along one row (at least one)."""
    return max(1, int(round(lot_length / max(0.5, float(bay_width)))))


def divider_positions(lot_length: float, bay_width: float) -> List[float]:
    """The y of every painted bay divider: ``count + 1`` lines, ends included."""
    count = bay_count(lot_length, bay_width)
    step = lot_length / count
    half = lot_length / 2.0
    return [-half + index * step for index in range(count + 1)]


def bays(lot_length: float, aisle_width: float, bay_depth: float,
         bay_width: float) -> List[Bay]:
    """Every bay of both rows: ``A01…`` (left) and ``B01…`` (right), from -Y."""
    rows: List[Bay] = []
    count = bay_count(lot_length, bay_width)
    step = lot_length / count
    half = lot_length / 2.0
    for side in (-1.0, 1.0):
        letter = ROW_LETTERS[side]
        for index in range(count):
            rows.append(Bay(
                side=side,
                index=index,
                label=f"{letter}{index + 1:02d}",
                x=side * (aisle_width / 2.0 + bay_depth / 2.0),
                y=-half + (index + 0.5) * step,
                depth=bay_depth,
                width=step,
            ))
    return rows


def pillar_positions(lot_length: float, aisle_width: float,
                     count: int) -> List[Tuple[float, float]]:
    """``(x, y)`` of every column: ``count`` per side, evenly spaced."""
    if count <= 0:
        return []
    positions: List[Tuple[float, float]] = []
    for side in (-1.0, 1.0):
        for index in range(count):
            positions.append((
                side * (aisle_width / 2.0 + PILLAR_OFFSET),
                -lot_length / 2.0 + (index + 0.5) * lot_length / count,
            ))
    return positions


def parked_cars(lot_bays: Sequence[Bay], pillars: Sequence[Tuple[float, float]],
                per_side: int) -> List[ParkedCar]:
    """The parked vehicles: ``per_side`` per row, spread over the free bays.

    A bay whose column would reach into a car stays empty (`PILLAR_CLEARANCE`),
    which is what real car parks look like - and it keeps the column out of the
    parked car's body.  The forms cycle through :data:`PARKED_FORMS`, so every
    bay gets the same vehicle on every rebuild.
    """
    if per_side <= 0:
        return []
    result: List[ParkedCar] = []
    for side in (-1.0, 1.0):
        column_y = [y for x, y in pillars if (x > 0) == (side > 0)]
        row = [bay for bay in lot_bays if bay.side == side]
        free = [bay for bay in row
                if all(abs(bay.y - y) >= PILLAR_CLEARANCE for y in column_y)]
        if not free:
            continue
        count = min(int(per_side), len(free))
        step = len(free) / count
        for slot in range(count):
            bay = free[int(slot * step)]
            length, width, height, paint = PARKED_FORMS[
                (bay.index + (0 if side < 0 else 2)) % len(PARKED_FORMS)]
            result.append(ParkedCar(
                bay=bay, length=min(length, bay.depth - 0.2), width=width,
                height=height, paint=paint, yaw=-90.0 * side))
    return result


def label_position(bay: Bay, aisle_width: float, inset: float = 0.75) -> Tuple[float, float]:
    """Where the bay's number is painted: in the aisle, in front of the bay.

    Not inside the bay: a parked car would hide it, and a number the camera can
    still read while driving past is what makes the clip comparable frame by
    frame.
    """
    return (bay.side * (aisle_width / 2.0 - inset), bay.y)
