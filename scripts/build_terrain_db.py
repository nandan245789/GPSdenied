"""Build Georeferenced Terrain Database from Nadir DJI Frames.

Processes all nadir (top-down) frames to create a terrain database for
real-time terrain-aided navigation. Each frame becomes a database entry with:
  - ORB descriptors (2000 per frame)
  - GPS position (lat, lon, alt from EXIF)
  - NED position (meters from mission origin)
  - Ground footprint (meters covered at 60m AGL)
  - Terrain classification
  - Texture/edge quality scores

Output: data/terrain_db.pkl — loaded by terrain_matcher.py at runtime.

Usage: python3 scripts/build_terrain_db.py
"""

import cv2
import numpy as np
import os
import sys
import pickle
import subprocess
import json
import time
from pathlib import Path
from math import radians, cos, sin, sqrt, atan2


# ─── Constants ───────────────────────────────────────────────────────────────

# DJI camera specs at 60m AGL
SENSOR_WIDTH_MM = 13.2         # DJI Mini/Air series sensor
SENSOR_HEIGHT_MM = 8.8
FOCAL_LENGTH_MM = 4.49         # Typical DJI wide angle
IMAGE_WIDTH_PX = 5472
IMAGE_HEIGHT_PX = 3648
FLIGHT_ALT_M = 60.0

# Ground sampling distance
GSD = (SENSOR_WIDTH_MM * FLIGHT_ALT_M) / (FOCAL_LENGTH_MM * IMAGE_WIDTH_PX)
FOOTPRINT_W = GSD * IMAGE_WIDTH_PX   # ~43.7m
FOOTPRINT_H = GSD * IMAGE_HEIGHT_PX  # ~29.1m

# ORB config — tuned for aerial terrain matching
ORB_FEATURES = 2000
ORB_SCALE = 1.2
ORB_LEVELS = 8
ORB_EDGE = 31
ORB_PATCH = 31

# Processing
RESIZE_DIM = 1024  # Resize to 1024px max for feature extraction speed


def haversine_m(lat1, lon1, lat2, lon2):
    """Distance in meters between two GPS points."""
    R = 6371000
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat/2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon/2)**2
    return R * 2 * atan2(sqrt(a), sqrt(1 - a))


def gps_to_ned(lat, lon, alt, origin_lat, origin_lon, origin_alt):
    """Convert GPS to NED (North-East-Down) relative to origin."""
    R = 6371000
    north = radians(lat - origin_lat) * R
    east = radians(lon - origin_lon) * R * cos(radians(origin_lat))
    down = -(alt - origin_alt)  # NED convention: down is positive
    return north, east, down


def get_exif_gps(filepath):
    """Extract GPS from macOS Spotlight metadata (fastest method)."""
    try:
        result = subprocess.run(
            ['mdls', '-raw', '-name', 'kMDItemLatitude', filepath],
            capture_output=True, text=True, timeout=2
        )
        lat_str = result.stdout.strip()
        lat = float(lat_str) if lat_str != '(null)' else None

        result = subprocess.run(
            ['mdls', '-raw', '-name', 'kMDItemLongitude', filepath],
            capture_output=True, text=True, timeout=2
        )
        lon_str = result.stdout.strip()
        lon = float(lon_str) if lon_str != '(null)' else None

        result = subprocess.run(
            ['mdls', '-raw', '-name', 'kMDItemAltitude', filepath],
            capture_output=True, text=True, timeout=2
        )
        alt_str = result.stdout.strip()
        alt = float(alt_str) if alt_str != '(null)' else None

        return lat, lon, alt
    except Exception:
        return None, None, None


def classify_terrain(img_bgr):
    """Classify terrain type from color distribution."""
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    veg_mask = cv2.inRange(hsv, (25, 30, 30), (85, 255, 255))
    veg_pct = np.count_nonzero(veg_mask) / veg_mask.size * 100
    gray_mask = cv2.inRange(hsv, (0, 0, 100), (180, 40, 255))
    urban_pct = np.count_nonzero(gray_mask) / gray_mask.size * 100
    soil_mask = cv2.inRange(hsv, (0, 40, 50), (25, 255, 200))
    soil_pct = np.count_nonzero(soil_mask) / soil_mask.size * 100

    if veg_pct > 60:
        return 'dense_vegetation'
    elif urban_pct > 30:
        return 'urban'
    elif soil_pct > 25:
        return 'barren_soil'
    elif veg_pct > 30 and (urban_pct > 10 or soil_pct > 10):
        return 'mixed_suburban'
    else:
        return 'mixed'


def main():
    base = '/Users/nandanhemaraju/Desktop/GPSdenied'
    nadir_dir = os.path.join(base, 'frames_nadir')
    out_dir = os.path.join(base, 'data')
    os.makedirs(out_dir, exist_ok=True)

    print()
    print('=' * 70)
    print('  🗺️  Building Georeferenced Terrain Database')
    print('=' * 70)
    print()
    print(f'  Camera: DJI {IMAGE_WIDTH_PX}×{IMAGE_HEIGHT_PX}')
    print(f'  GSD:    {GSD*100:.1f} cm/pixel')
    print(f'  Footprint: {FOOTPRINT_W:.1f}m × {FOOTPRINT_H:.1f}m per frame')
    print(f'  ORB:    {ORB_FEATURES} features/frame')
    print()

    # Initialize ORB
    orb = cv2.ORB_create(
        nfeatures=ORB_FEATURES,
        scaleFactor=ORB_SCALE,
        nlevels=ORB_LEVELS,
        edgeThreshold=ORB_EDGE,
        patchSize=ORB_PATCH
    )

    # Collect all nadir frames
    all_frames = []
    for set_name in sorted(os.listdir(nadir_dir)):
        set_path = os.path.join(nadir_dir, set_name)
        if not os.path.isdir(set_path):
            continue
        for fname in sorted(os.listdir(set_path)):
            if fname.upper().endswith('.JPG'):
                all_frames.append({
                    'path': os.path.join(set_path, fname),
                    'set': set_name,
                    'filename': fname
                })

    print(f'  Found {len(all_frames)} nadir frames across '
          f'{len(set(f["set"] for f in all_frames))} sets')
    print()

    # ── Phase 1: Extract GPS from all frames to find origin ──────────────
    print('  Phase 1: Reading GPS metadata...')
    gps_data = []
    for f in all_frames:
        lat, lon, alt = get_exif_gps(f['path'])
        f['lat'] = lat
        f['lon'] = lon
        f['alt'] = alt
        if lat and lon:
            gps_data.append((lat, lon, alt or 832.0))

    # Mission origin = centroid of all GPS points
    lats = [g[0] for g in gps_data]
    lons = [g[1] for g in gps_data]
    alts = [g[2] for g in gps_data]
    origin_lat = np.mean(lats)
    origin_lon = np.mean(lons)
    origin_alt = np.mean(alts)

    print(f'  GPS origin: {origin_lat:.6f}°N, {origin_lon:.6f}°E, {origin_alt:.1f}m ASL')
    print(f'  GPS spread: {haversine_m(min(lats), min(lons), max(lats), max(lons)):.0f}m diagonal')
    print()

    # ── Phase 2: Process each frame ──────────────────────────────────────
    print('  Phase 2: Extracting features and building database...')
    print()
    print(f'  {"#":>4}  {"Frame":>15}  {"Set":>14}  '
          f'{"N(m)":>7}  {"E(m)":>7}  {"Feat":>5}  {"Tex":>7}  {"Type":<18}')
    print(f'  {"─"*4}  {"─"*15}  {"─"*14}  '
          f'{"─"*7}  {"─"*7}  {"─"*5}  {"─"*7}  {"─"*18}')

    db_entries = []
    all_descriptors = []
    start_time = time.time()

    for i, f in enumerate(all_frames):
        if f['lat'] is None or f['lon'] is None:
            continue

        # Read and resize
        img = cv2.imread(f['path'])
        if img is None:
            continue

        h, w = img.shape[:2]
        scale = RESIZE_DIM / max(h, w)
        img_small = cv2.resize(img, (int(w * scale), int(h * scale)))
        gray = cv2.cvtColor(img_small, cv2.COLOR_BGR2GRAY)

        # ORB extraction
        kp, des = orb.detectAndCompute(gray, None)
        n_features = len(kp) if kp else 0

        if des is None or n_features < 50:
            continue

        # Texture score
        texture = float(np.var(cv2.Laplacian(gray, cv2.CV_64F)))

        # Terrain classification
        terrain = classify_terrain(img_small)

        # NED position
        north, east, down = gps_to_ned(
            f['lat'], f['lon'], f['alt'] or origin_alt,
            origin_lat, origin_lon, origin_alt
        )

        # Keypoint positions in normalized coordinates [0,1]
        kp_positions = np.array([(k.pt[0] / img_small.shape[1],
                                   k.pt[1] / img_small.shape[0]) for k in kp],
                                 dtype=np.float32)

        # Database entry
        entry = {
            'frame_id': i,
            'filename': f['filename'],
            'set_name': f['set'],
            'gps': {
                'lat': f['lat'],
                'lon': f['lon'],
                'alt': f['alt'] or origin_alt
            },
            'ned': {
                'north': round(north, 2),
                'east': round(east, 2),
                'down': round(down, 2)
            },
            'footprint': {
                'width_m': round(FOOTPRINT_W, 1),
                'height_m': round(FOOTPRINT_H, 1),
                'gsd_cm': round(GSD * 100, 2)
            },
            'features': {
                'n_keypoints': n_features,
                'descriptors': des,           # uint8 array [N×32]
                'positions': kp_positions,     # float32 array [N×2]
            },
            'quality': {
                'texture_score': round(texture, 1),
                'terrain_type': terrain,
            }
        }

        db_entries.append(entry)
        all_descriptors.append(des)

        # Print progress every 10 frames
        if i % 10 == 0 or i == len(all_frames) - 1:
            print(f'  {i+1:>4}  {f["filename"]:>15}  {f["set"]:>14}  '
                  f'{north:>7.1f}  {east:>7.1f}  {n_features:>5}  '
                  f'{texture:>7.0f}  {terrain:<18}')

    elapsed = time.time() - start_time

    # ── Phase 3: Build spatial index ─────────────────────────────────────
    print()
    print('  Phase 3: Building spatial index...')

    # Simple grid-based spatial index for fast lookup
    # Divide area into 10m × 10m cells
    CELL_SIZE = 10.0
    spatial_index = {}
    for idx, entry in enumerate(db_entries):
        n, e = entry['ned']['north'], entry['ned']['east']
        cell_n = int(n // CELL_SIZE)
        cell_e = int(e // CELL_SIZE)
        key = (cell_n, cell_e)
        if key not in spatial_index:
            spatial_index[key] = []
        spatial_index[key].append(idx)

    print(f'  Grid cells: {len(spatial_index)} ({CELL_SIZE}m resolution)')

    # ── Phase 4: Build FLANN index for fast descriptor matching ──────────
    print('  Phase 4: Building FLANN descriptor index...')

    # Stack all descriptors into one giant array
    all_des = np.vstack(all_descriptors).astype(np.uint8)
    # Map each descriptor back to its frame
    desc_to_frame = []
    for idx, entry in enumerate(db_entries):
        n_kp = entry['features']['n_keypoints']
        desc_to_frame.extend([idx] * n_kp)
    desc_to_frame = np.array(desc_to_frame, dtype=np.int32)

    print(f'  Total descriptors: {len(all_des):,}')
    print(f'  Descriptor matrix: {all_des.shape} ({all_des.nbytes / 1024 / 1024:.1f} MB)')

    # ── Phase 5: Save database ───────────────────────────────────────────
    print()
    print('  Phase 5: Saving terrain database...')

    # Compute metadata
    positions = np.array([(e['ned']['north'], e['ned']['east']) for e in db_entries])

    db = {
        'version': '1.0',
        'created': time.strftime('%Y-%m-%d %H:%M:%S'),
        'origin': {
            'lat': origin_lat,
            'lon': origin_lon,
            'alt': origin_alt,
            'description': 'Centroid of all nadir frames'
        },
        'camera': {
            'sensor_mm': (SENSOR_WIDTH_MM, SENSOR_HEIGHT_MM),
            'focal_mm': FOCAL_LENGTH_MM,
            'resolution': (IMAGE_WIDTH_PX, IMAGE_HEIGHT_PX),
            'gsd_cm': round(GSD * 100, 2),
            'footprint_m': (round(FOOTPRINT_W, 1), round(FOOTPRINT_H, 1))
        },
        'coverage': {
            'n_frames': len(db_entries),
            'north_range': (round(float(positions[:, 0].min()), 1),
                           round(float(positions[:, 0].max()), 1)),
            'east_range': (round(float(positions[:, 1].min()), 1),
                          round(float(positions[:, 1].max()), 1)),
            'area_m2': round(float(
                (positions[:, 0].max() - positions[:, 0].min()) *
                (positions[:, 1].max() - positions[:, 1].min())
            ), 0),
            'terrain_distribution': {}
        },
        'entries': db_entries,
        'spatial_index': spatial_index,
        'flann': {
            'descriptors': all_des,
            'desc_to_frame': desc_to_frame,
        }
    }

    # Terrain distribution
    for entry in db_entries:
        t = entry['quality']['terrain_type']
        db['coverage']['terrain_distribution'][t] = \
            db['coverage']['terrain_distribution'].get(t, 0) + 1

    # Save
    db_path = os.path.join(out_dir, 'terrain_db.pkl')
    with open(db_path, 'wb') as f:
        pickle.dump(db, f, protocol=pickle.HIGHEST_PROTOCOL)

    db_size = os.path.getsize(db_path)

    # Also save a lightweight metadata JSON (no descriptors)
    meta = {k: v for k, v in db.items() if k not in ('entries', 'spatial_index', 'flann')}
    meta['entries_count'] = len(db_entries)
    meta['db_file_size_mb'] = round(db_size / 1024 / 1024, 1)
    meta_path = os.path.join(out_dir, 'terrain_db_meta.json')
    with open(meta_path, 'w') as f:
        json.dump(meta, f, indent=2, default=str)

    # ── Report ───────────────────────────────────────────────────────────
    print()
    print('=' * 70)
    print('  📊  TERRAIN DATABASE REPORT')
    print('=' * 70)
    print()
    print(f'  Database file:     {db_path}')
    print(f'  Database size:     {db_size / 1024 / 1024:.1f} MB')
    print(f'  Metadata file:     {meta_path}')
    print()
    print(f'  Processing time:   {elapsed:.1f}s ({elapsed/len(db_entries)*1000:.0f}ms/frame)')
    print(f'  Frames processed:  {len(db_entries)}/{len(all_frames)}')
    print(f'  Total keypoints:   {len(all_des):,}')
    print(f'  Descriptor size:   {all_des.nbytes / 1024 / 1024:.1f} MB')
    print()
    print(f'  Origin:            {origin_lat:.6f}°N, {origin_lon:.6f}°E')
    print(f'  Coverage (N-S):    {positions[:,0].min():.1f}m to {positions[:,0].max():.1f}m '
          f'({positions[:,0].max() - positions[:,0].min():.0f}m)')
    print(f'  Coverage (E-W):    {positions[:,1].min():.1f}m to {positions[:,1].max():.1f}m '
          f'({positions[:,1].max() - positions[:,1].min():.0f}m)')
    print(f'  Coverage area:     {db["coverage"]["area_m2"]:.0f} m² '
          f'({db["coverage"]["area_m2"]/10000:.2f} ha)')
    print(f'  Grid cells:        {len(spatial_index)} ({CELL_SIZE}m × {CELL_SIZE}m)')
    print(f'  Terrain:           {db["coverage"]["terrain_distribution"]}')
    print()
    print(f'  GSD:               {GSD*100:.1f} cm/pixel')
    print(f'  Frame footprint:   {FOOTPRINT_W:.1f}m × {FOOTPRINT_H:.1f}m')
    print()

    # Matching performance estimate
    avg_overlap = FOOTPRINT_W * 0.6  # 60% overlap typical for DJI mapping
    est_accuracy = GSD * 100 * 5     # ~5 pixels of matching error
    print(f'  ── Expected Matching Performance ──')
    print(f'  Est. matching accuracy: ±{est_accuracy:.0f}cm (at {GSD*100:.1f}cm GSD)')
    print(f'  Avg frame overlap:      ~{avg_overlap:.0f}m')
    print(f'  Max VIO drift before fail: ~{FOOTPRINT_W*0.3:.0f}m '
          f'(30% of footprint)')
    print()
    print('  ✅  Terrain database ready for terrain_matcher.py')
    print('=' * 70)


if __name__ == '__main__':
    main()
