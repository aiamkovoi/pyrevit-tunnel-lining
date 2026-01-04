# -*- coding: utf-8 -*-
__title__ = "Tunnel Adaptive"
__doc__ = """Version = 7.0
Date    = 04.01.2025
________________________________________________________________
Description:

Places adaptive tunnel components along segments from Linked CAD.
Click on ANY line - script automatically finds full connected chain.
Supports multiple families and Arc tessellation.

________________________________________________________________
How-To:

1. Run the script
2. Enter family names (comma-separated)
3. Click on ANY line segment (script finds full chain in both directions)
4. Adaptive components will be placed along the entire alignment

________________________________________________________________
Author: Aleksandr Iamkovoi"""

from Autodesk.Revit.DB import *
from Autodesk.Revit.UI.Selection import ObjectType, ISelectionFilter
from Autodesk.Revit.Exceptions import OperationCanceledException

import math
import time

from pyrevit import revit, forms

# ==================================================
# CONFIGURATION
# ==================================================
doc = __revit__.ActiveUIDocument.Document
uidoc = __revit__.ActiveUIDocument

DEFAULT_FAMILIES = "Crown, Base"

MIN_LENGTH_MM = 100.0
ROUND_DIGITS = 2
ANGLE_OFFSET_DEG = -90.0
ARC_SEGMENT_LENGTH_MM = 10000.0
MAX_GAP_FT = 50.0

MIN_LENGTH_FT = MIN_LENGTH_MM / 304.8
ARC_SEGMENT_LENGTH_FT = ARC_SEGMENT_LENGTH_MM / 304.8
PI_DIV_180 = math.pi / 180.0


# ==================================================
# FILTER
# ==================================================
class LinkedCADFilter(ISelectionFilter):
    def AllowElement(self, elem):
        return isinstance(elem, ImportInstance)

    def AllowReference(self, ref, pt):
        return True


# ==================================================
# GEOMETRY FUNCTIONS
# ==================================================
def normalize_angle(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def distance_point_to_segment(px, py, pz, s0x, s0y, s0z, s1x, s1y, s1z):
    """Optimized distance from point to segment"""
    dx, dy, dz = s1x - s0x, s1y - s0y, s1z - s0z
    len_sq = dx * dx + dy * dy + dz * dz

    if len_sq < 0.0001:
        return math.sqrt((px - s0x) ** 2 + (py - s0y) ** 2 + (pz - s0z) ** 2)

    t = max(0, min(1, ((px - s0x) * dx + (py - s0y) * dy + (pz - s0z) * dz) / len_sq))

    cx, cy, cz = s0x + t * dx, s0y + t * dy, s0z + t * dz
    return math.sqrt((px - cx) ** 2 + (py - cy) ** 2 + (pz - cz) ** 2)


def get_all_curves_from_cad(cad_instance):
    """Extract all curves from CAD as (start, end) tuples"""
    segments = []
    opt = Options()
    opt.ComputeReferences = True
    opt.DetailLevel = ViewDetailLevel.Fine

    geom = cad_instance.get_Geometry(opt)
    if not geom:
        return segments

    for geom_obj in geom:
        if not isinstance(geom_obj, GeometryInstance):
            continue

        for inst_obj in geom_obj.GetInstanceGeometry():
            if isinstance(inst_obj, Line):
                segments.append((inst_obj.GetEndPoint(0), inst_obj.GetEndPoint(1)))

            elif isinstance(inst_obj, PolyLine):
                coords = list(inst_obj.GetCoordinates())
                for i in range(len(coords) - 1):
                    segments.append((coords[i], coords[i + 1]))

            elif isinstance(inst_obj, Arc) or isinstance(inst_obj, Curve):
                curve_length = inst_obj.Length
                num_segs = max(2, int(math.ceil(curve_length / ARC_SEGMENT_LENGTH_FT)))
                p0, p1 = inst_obj.GetEndParameter(0), inst_obj.GetEndParameter(1)

                points = [inst_obj.Evaluate(p0 + (p1 - p0) * (float(i) / num_segs), False)
                          for i in range(num_segs + 1)]

                for i in range(len(points) - 1):
                    segments.append((points[i], points[i + 1]))

    return segments


def find_closest_segment(segments, click_pt):
    """Find closest segment to click point"""
    if not segments:
        return None

    px, py, pz = click_pt.X, click_pt.Y, click_pt.Z
    min_dist = float('inf')
    closest_idx = None

    for i, seg in enumerate(segments):
        s0, s1 = seg
        dist = distance_point_to_segment(px, py, pz, s0.X, s0.Y, s0.Z, s1.X, s1.Y, s1.Z)
        if dist < min_dist:
            min_dist = dist
            closest_idx = i

    return closest_idx


def build_chain_one_direction(start_point, all_segments, used_indices):
    """Build chain in one direction from start_point"""
    chain = []
    current_end = start_point

    for _ in range(len(all_segments)):
        best_idx = None
        best_dist = float('inf')
        best_reverse = False

        ex, ey, ez = current_end.X, current_end.Y, current_end.Z

        for i, seg in enumerate(all_segments):
            if i in used_indices:
                continue

            s0, s1 = seg
            d0 = (s0.X - ex) ** 2 + (s0.Y - ey) ** 2 + (s0.Z - ez) ** 2
            d1 = (s1.X - ex) ** 2 + (s1.Y - ey) ** 2 + (s1.Z - ez) ** 2

            if d0 < best_dist:
                best_dist, best_idx, best_reverse = d0, i, False
            if d1 < best_dist:
                best_dist, best_idx, best_reverse = d1, i, True

        if best_idx is None or best_dist > MAX_GAP_FT * MAX_GAP_FT:
            break

        used_indices.add(best_idx)
        seg = all_segments[best_idx]

        if best_reverse:
            chain.append((seg[1], seg[0]))
            current_end = seg[0]
        else:
            chain.append(seg)
            current_end = seg[1]

    return chain


def build_full_chain(first_seg_idx, all_segments):
    """Build complete chain in both directions from clicked segment"""
    first_seg = all_segments[first_seg_idx]
    used_indices = {first_seg_idx}

    # Build chain forward (from end of first segment)
    forward_chain = build_chain_one_direction(first_seg[1], all_segments, used_indices)

    # Build chain backward (from start of first segment)
    backward_chain = build_chain_one_direction(first_seg[0], all_segments, used_indices)

    # Reverse backward chain and flip each segment, then combine
    # backward goes: first_seg[0] -> ... -> end
    # we need: end -> ... -> first_seg[0] -> first_seg[1] -> ... -> forward_end

    reversed_backward = []
    for seg in reversed(backward_chain):
        # Flip segment direction
        reversed_backward.append((seg[1], seg[0]))

    # Combine: reversed_backward + first_seg + forward_chain
    full_chain = reversed_backward + [first_seg] + forward_chain

    return full_chain


def calculate_angles(segments):
    """Calculate rotation angles for segments"""
    raw_angles = []
    valid_segs = []

    for sp, ep in segments:
        dx, dy = ep.X - sp.X, ep.Y - sp.Y
        length = math.sqrt(dx * dx + dy * dy)

        if length < MIN_LENGTH_FT:
            continue

        angle = math.degrees(math.atan2(dy, dx)) + ANGLE_OFFSET_DEG
        angle = normalize_angle(angle)

        raw_angles.append(angle)
        valid_segs.append((sp, ep))

    if not raw_angles:
        return [], [], []

    # Smooth
    smoothed = [raw_angles[0]]
    for i in range(1, len(raw_angles)):
        smoothed.append((raw_angles[i - 1] + raw_angles[i]) / 2.0)

    smoothed = [round(a, ROUND_DIGITS) for a in smoothed]

    return smoothed, smoothed[1:] + [smoothed[-1]], valid_segs


def get_family_symbols(names):
    """Get family symbols by names"""
    all_syms = {}
    for sym in FilteredElementCollector(doc).OfClass(FamilySymbol):
        try:
            if sym.Family:
                all_syms[sym.Family.Name] = sym
        except:
            pass

    found, missing = [], []
    for n in names:
        n = n.strip()
        if n in all_syms:
            found.append((n, all_syms[n]))
        elif n:
            missing.append(n)

    return found, missing


# ==================================================
# MAIN
# ==================================================
def main():
    start_time = time.time()

    # Get families
    family_input = forms.ask_for_string(
        default=DEFAULT_FAMILIES,
        prompt="Enter family names (comma-separated):",
        title="Tunnel Adaptive Placement"
    )
    if not family_input:
        return

    symbols, not_found = get_family_symbols(family_input.split(","))

    if not_found:
        msg = "Families not found:\n- " + "\n- ".join(not_found)
        if not symbols:
            forms.alert(msg, exitscript=True)
            return
        if not forms.alert(msg + "\n\nContinue?", yes=True, no=True):
            return

    if not symbols:
        forms.alert("No valid families!", exitscript=True)
        return

    # Select CAD line
    try:
        ref = uidoc.Selection.PickObject(
            ObjectType.PointOnElement,
            LinkedCADFilter(),
            "Click on ANY line segment (full chain will be found)"
        )
        cad = doc.GetElement(ref.ElementId)
        if not isinstance(cad, ImportInstance):
            forms.alert("Not a CAD link!", exitscript=True)
            return
        click_pt = ref.GlobalPoint
    except OperationCanceledException:
        return
    except:
        return

    # Get segments and build chain
    all_segs = get_all_curves_from_cad(cad)
    if not all_segs:
        forms.alert("No lines in CAD!", exitscript=True)
        return

    first_seg_idx = find_closest_segment(all_segs, click_pt)
    if first_seg_idx is None:
        forms.alert("No segment found!", exitscript=True)
        return

    chain = build_full_chain(first_seg_idx, all_segs)
    if len(chain) < 1:
        forms.alert("Could not build chain!", exitscript=True)
        return

    # Calculate angles
    rot_start, rot_end, valid_segs = calculate_angles(chain)
    if not valid_segs:
        forms.alert("No valid segments!", exitscript=True)
        return

    # Place components
    results = {}

    with revit.Transaction("Place Tunnel Segments"):
        # Activate all symbols first
        for _, sym in symbols:
            if not sym.IsActive:
                sym.Activate()
        doc.Regenerate()

        for fam_name, sym in symbols:
            created, errors = 0, 0

            for i, (sp, ep) in enumerate(valid_segs):
                try:
                    inst = AdaptiveComponentInstanceUtils.CreateAdaptiveComponentInstance(doc, sym)
                    pts = AdaptiveComponentInstanceUtils.GetInstancePlacementPointElementRefIds(inst)

                    if pts.Count >= 2:
                        doc.GetElement(pts[0]).Position = sp
                        doc.GetElement(pts[1]).Position = ep
                    elif pts.Count == 1:
                        doc.GetElement(pts[0]).Position = XYZ(
                            (sp.X + ep.X) / 2, (sp.Y + ep.Y) / 2, (sp.Z + ep.Z) / 2
                        )

                    # Set rotation parameters
                    param = inst.LookupParameter("Rotation_start")
                    if param and not param.IsReadOnly:
                        param.Set(rot_start[i] * PI_DIV_180)

                    param = inst.LookupParameter("Rotation_end")
                    if param and not param.IsReadOnly:
                        param.Set(rot_end[i] * PI_DIV_180)

                    created += 1
                except:
                    errors += 1

            results[fam_name] = (created, errors)

    # Summary
    elapsed = time.time() - start_time

    msg = "Completed!\n\n"
    msg += "Segments: {}\n\n".format(len(valid_segs))

    total_created = 0
    for name, (c, e) in results.items():
        msg += "{}: {}".format(name, c)
        if e: msg += " ({} errors)".format(e)
        msg += "\n"
        total_created += c

    msg += "\nTotal: {} components\nTime: {:.2f} sec".format(total_created, elapsed)
    forms.alert(msg, title="Tunnel Adaptive Placement")


if __name__ == "__main__":
    main()