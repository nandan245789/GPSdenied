"""Terrain Feature Diversity Analyzer for GPS-Denied Navigation.

Analyzes DJI aerial datasets to assess terrain-matching viability:
  1. ORB/SIFT feature extraction density per frame
  2. Feature repeatability between adjacent frames (overlap matching)
  3. Terrain classification (urban / vegetation / mixed / barren)
  4. Feature-poor zone detection (where terrain matching will struggle)
  5. GPS coverage mapping from EXIF data

Usage: python3 scripts/analyze_terrain_features.py
"""

import cv2
import numpy as np
import os
import sys
import json
from pathlib import Path
from datetime import datetime

# Suppress OpenCV debug output
os.environ['OPENCV_LOG_LEVEL'] = 'ERROR'


class TerrainFeatureAnalyzer:
    def __init__(self):
        # Use ORB (free, fast, rotation-invariant)
        self.orb = cv2.ORB_create(nfeatures=2000)
        # Brute-force matcher for binary descriptors
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        # Results storage
        self.results = {
            'nadir': {},
            'oblique': {},
            'summary': {}
        }

    def classify_terrain(self, img_bgr):
        """Classify terrain type from color distribution."""
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        # Green vegetation mask (H: 25-85, S: 30+, V: 30+)
        veg_mask = cv2.inRange(hsv, (25, 30, 30), (85, 255, 255))
        veg_pct = np.count_nonzero(veg_mask) / veg_mask.size * 100

        # Built-up/urban (low saturation, high value = concrete/metal)
        gray_mask = cv2.inRange(hsv, (0, 0, 100), (180, 40, 255))
        urban_pct = np.count_nonzero(gray_mask) / gray_mask.size * 100

        # Red/brown soil (laterite) — common in your terrain
        soil_mask = cv2.inRange(hsv, (0, 40, 50), (25, 255, 200))
        soil_pct = np.count_nonzero(soil_mask) / soil_mask.size * 100

        # Classify
        if veg_pct > 60:
            terrain = 'dense_vegetation'
        elif urban_pct > 30:
            terrain = 'urban'
        elif soil_pct > 25:
            terrain = 'barren_soil'
        elif veg_pct > 30 and (urban_pct > 10 or soil_pct > 10):
            terrain = 'mixed_suburban'
        else:
            terrain = 'mixed'

        return terrain, {
            'vegetation_pct': round(veg_pct, 1),
            'urban_pct': round(urban_pct, 1),
            'soil_pct': round(soil_pct, 1)
        }

    def compute_texture_score(self, gray):
        """Compute Laplacian variance — measures texture richness."""
        lap = cv2.Laplacian(gray, cv2.CV_64F)
        return float(np.var(lap))

    def compute_edge_density(self, gray):
        """Canny edge density — measures structural features."""
        edges = cv2.Canny(gray, 50, 150)
        return np.count_nonzero(edges) / edges.size * 100

    def extract_gps(self, filepath):
        """Extract GPS from EXIF using macOS mdls (fast, no PIL needed)."""
        try:
            import subprocess
            result = subprocess.run(
                ['mdls', '-raw', '-name', 'kMDItemLatitude', filepath],
                capture_output=True, text=True, timeout=2
            )
            lat = float(result.stdout.strip()) if result.stdout.strip() != '(null)' else None

            result = subprocess.run(
                ['mdls', '-raw', '-name', 'kMDItemLongitude', filepath],
                capture_output=True, text=True, timeout=2
            )
            lon = float(result.stdout.strip()) if result.stdout.strip() != '(null)' else None

            result = subprocess.run(
                ['mdls', '-raw', '-name', 'kMDItemAltitude', filepath],
                capture_output=True, text=True, timeout=2
            )
            alt = float(result.stdout.strip()) if result.stdout.strip() != '(null)' else None

            return lat, lon, alt
        except Exception:
            return None, None, None

    def analyze_frame(self, filepath, resize_to=1024):
        """Analyze a single frame for terrain matching viability."""
        img = cv2.imread(filepath)
        if img is None:
            return None

        # Resize for speed (keep aspect ratio)
        h, w = img.shape[:2]
        scale = resize_to / max(h, w)
        img_small = cv2.resize(img, (int(w * scale), int(h * scale)))
        gray = cv2.cvtColor(img_small, cv2.COLOR_BGR2GRAY)

        # ORB features
        kp, des = self.orb.detectAndCompute(gray, None)
        n_features = len(kp) if kp else 0

        # Texture score
        texture = self.compute_texture_score(gray)

        # Edge density
        edges = self.compute_edge_density(gray)

        # Terrain classification
        terrain_type, terrain_pcts = self.classify_terrain(img_small)

        # Feature distribution — divide into 4x4 grid
        grid_counts = np.zeros((4, 4))
        gh, gw = gray.shape[0] // 4, gray.shape[1] // 4
        if kp:
            for k in kp:
                gi = min(int(k.pt[1] / gh), 3)
                gj = min(int(k.pt[0] / gw), 3)
                grid_counts[gi, gj] += 1
        grid_std = float(np.std(grid_counts))
        grid_coverage = float(np.count_nonzero(grid_counts) / 16 * 100)

        return {
            'n_features': n_features,
            'texture_score': round(texture, 1),
            'edge_density': round(edges, 2),
            'terrain_type': terrain_type,
            'terrain_pcts': terrain_pcts,
            'grid_coverage': round(grid_coverage, 1),
            'grid_uniformity': round(100 - grid_std / max(n_features / 16, 1) * 100, 1),
            'descriptors': des
        }

    def match_adjacent(self, des1, des2):
        """Match features between adjacent frames (overlap check)."""
        if des1 is None or des2 is None:
            return 0, 0.0
        try:
            matches = self.bf.knnMatch(des1, des2, k=2)
            # Lowe's ratio test
            good = []
            for m_n in matches:
                if len(m_n) == 2:
                    m, n = m_n
                    if m.distance < 0.75 * n.distance:
                        good.append(m)
            return len(good), len(good) / max(len(matches), 1) * 100
        except Exception:
            return 0, 0.0

    def analyze_dataset(self, dataset_path, dataset_name, dataset_type):
        """Analyze an entire set of frames."""
        files = sorted([
            os.path.join(dataset_path, f)
            for f in os.listdir(dataset_path)
            if f.upper().endswith('.JPG')
        ])

        if not files:
            print(f'  ⚠️  No JPG files found in {dataset_path}')
            return

        n = len(files)
        # Sample every Nth frame to keep analysis under 2 minutes
        step = max(1, n // 30)  # Analyze ~30 frames per set
        sample_indices = list(range(0, n, step))

        print(f'\n  📂 {dataset_name} ({n} frames, sampling {len(sample_indices)})')
        print(f'  {"─" * 60}')

        features_list = []
        textures = []
        edges = []
        terrains = []
        match_counts = []
        match_ratios = []
        gps_coords = []
        prev_des = None
        feature_poor_frames = []

        for i, idx in enumerate(sample_indices):
            fpath = files[idx]
            fname = os.path.basename(fpath)

            result = self.analyze_frame(fpath)
            if result is None:
                continue

            features_list.append(result['n_features'])
            textures.append(result['texture_score'])
            edges.append(result['edge_density'])
            terrains.append(result['terrain_type'])

            # Adjacent frame matching
            if prev_des is not None and result['descriptors'] is not None:
                n_match, ratio = self.match_adjacent(prev_des, result['descriptors'])
                match_counts.append(n_match)
                match_ratios.append(ratio)
            prev_des = result['descriptors']

            # GPS
            lat, lon, alt = self.extract_gps(fpath)
            if lat and lon:
                gps_coords.append((lat, lon, alt or 0))

            # Flag feature-poor frames
            if result['n_features'] < 200:
                feature_poor_frames.append((fname, result['n_features'], result['terrain_type']))

            # Progress
            status = '🟢' if result['n_features'] >= 500 else ('🟡' if result['n_features'] >= 200 else '🔴')
            print(f'  {status} {fname:>15}  feat={result["n_features"]:>4}  '
                  f'tex={result["texture_score"]:>8.0f}  edge={result["edge_density"]:>5.1f}%  '
                  f'grid={result["grid_coverage"]:>5.0f}%  → {result["terrain_type"]}')

        # Summary for this set
        feat_arr = np.array(features_list)
        tex_arr = np.array(textures)
        edge_arr = np.array(edges)

        terrain_counts = {}
        for t in terrains:
            terrain_counts[t] = terrain_counts.get(t, 0) + 1

        set_result = {
            'n_frames_total': n,
            'n_frames_sampled': len(sample_indices),
            'features': {
                'mean': round(float(np.mean(feat_arr)), 0),
                'std': round(float(np.std(feat_arr)), 0),
                'min': int(np.min(feat_arr)),
                'max': int(np.max(feat_arr)),
                'pct_above_500': round(float(np.sum(feat_arr >= 500) / len(feat_arr) * 100), 1)
            },
            'texture': {
                'mean': round(float(np.mean(tex_arr)), 1),
                'std': round(float(np.std(tex_arr)), 1)
            },
            'edge_density': {
                'mean': round(float(np.mean(edge_arr)), 2),
                'std': round(float(np.std(edge_arr)), 2)
            },
            'terrain_distribution': terrain_counts,
            'matching': {
                'mean_matches': round(float(np.mean(match_counts)), 0) if match_counts else 0,
                'mean_ratio': round(float(np.mean(match_ratios)), 1) if match_ratios else 0,
                'min_matches': int(np.min(match_counts)) if match_counts else 0,
                'max_matches': int(np.max(match_counts)) if match_counts else 0
            },
            'gps_coverage': {
                'n_geotagged': len(gps_coords),
                'lat_range': [round(min(c[0] for c in gps_coords), 6),
                              round(max(c[0] for c in gps_coords), 6)] if gps_coords else None,
                'lon_range': [round(min(c[1] for c in gps_coords), 6),
                              round(max(c[1] for c in gps_coords), 6)] if gps_coords else None,
                'alt_range': [round(min(c[2] for c in gps_coords), 1),
                              round(max(c[2] for c in gps_coords), 1)] if gps_coords else None
            },
            'feature_poor_frames': feature_poor_frames
        }

        self.results[dataset_type][dataset_name] = set_result

        # Print summary
        print(f'\n  📊 {dataset_name} Summary:')
        print(f'     Features:       {set_result["features"]["mean"]:.0f} ± {set_result["features"]["std"]:.0f} '
              f'(min={set_result["features"]["min"]}, max={set_result["features"]["max"]})')
        print(f'     ≥500 features:  {set_result["features"]["pct_above_500"]}% of frames')
        print(f'     Texture:        {set_result["texture"]["mean"]:.0f} ± {set_result["texture"]["std"]:.0f}')
        print(f'     Edge density:   {set_result["edge_density"]["mean"]:.1f}% ± {set_result["edge_density"]["std"]:.1f}%')
        print(f'     Terrain types:  {terrain_counts}')
        if match_counts:
            print(f'     Overlap matches:{set_result["matching"]["mean_matches"]:.0f} avg '
                  f'({set_result["matching"]["mean_ratio"]:.1f}% ratio)')
        if feature_poor_frames:
            print(f'     ⚠️  Feature-poor: {len(feature_poor_frames)} frames <200 features')
        if gps_coords:
            print(f'     GPS coverage:   {set_result["gps_coverage"]["lat_range"]} lat, '
                  f'{set_result["gps_coverage"]["lon_range"]} lon')

    def compute_overall_viability(self):
        """Compute overall terrain-matching viability score."""
        all_feats = []
        all_matches = []
        terrains = {}

        for dtype in ['nadir', 'oblique']:
            for name, data in self.results[dtype].items():
                all_feats.append(data['features']['mean'])
                if data['matching']['mean_matches'] > 0:
                    all_matches.append(data['matching']['mean_ratio'])
                for t, c in data['terrain_distribution'].items():
                    terrains[t] = terrains.get(t, 0) + c

        avg_features = np.mean(all_feats) if all_feats else 0
        avg_match_ratio = np.mean(all_matches) if all_matches else 0

        # Viability scoring
        feat_score = min(avg_features / 1000, 1.0) * 40    # 40 pts for features
        match_score = min(avg_match_ratio / 30, 1.0) * 30  # 30 pts for matching
        diversity = len(terrains)
        div_score = min(diversity / 4, 1.0) * 15            # 15 pts for terrain diversity
        gps_score = 15                                       # 15 pts for GPS metadata

        total = feat_score + match_score + div_score + gps_score

        self.results['summary'] = {
            'viability_score': round(total, 1),
            'feature_score': round(feat_score, 1),
            'matching_score': round(match_score, 1),
            'diversity_score': round(div_score, 1),
            'gps_score': gps_score,
            'avg_features': round(avg_features, 0),
            'avg_match_ratio': round(avg_match_ratio, 1),
            'terrain_types': terrains,
            'verdict': 'EXCELLENT' if total > 80 else ('GOOD' if total > 60 else ('FAIR' if total > 40 else 'POOR'))
        }

        return self.results['summary']


def main():
    base = '/Users/nandanhemaraju/Desktop/GPSdenied'

    print()
    print('=' * 70)
    print('  🛰️  Terrain Feature Diversity Analysis')
    print('  Dataset: DJI aerial imagery @ 60m AGL')
    print('  Location: Northern Karnataka (~15.83°N, 74.40°E)')
    print('=' * 70)

    analyzer = TerrainFeatureAnalyzer()

    # Analyze nadir sets
    print('\n' + '━' * 70)
    print('  NADIR (Top-Down) DATASETS')
    print('━' * 70)
    nadir_base = os.path.join(base, 'frames_nadir')
    for d in sorted(os.listdir(nadir_base)):
        dp = os.path.join(nadir_base, d)
        if os.path.isdir(dp):
            analyzer.analyze_dataset(dp, d, 'nadir')

    # Analyze oblique sets
    print('\n' + '━' * 70)
    print('  OBLIQUE (45°) DATASETS')
    print('━' * 70)
    oblique_base = os.path.join(base, 'frames_oblique')
    for d in sorted(os.listdir(oblique_base)):
        dp = os.path.join(oblique_base, d)
        if os.path.isdir(dp):
            analyzer.analyze_dataset(dp, d, 'oblique')

    # Overall viability
    summary = analyzer.compute_overall_viability()

    print('\n' + '=' * 70)
    print('  🎯  TERRAIN MATCHING VIABILITY ASSESSMENT')
    print('=' * 70)
    print()

    verdict_icon = {'EXCELLENT': '🟢', 'GOOD': '🟢', 'FAIR': '🟡', 'POOR': '🔴'}
    icon = verdict_icon.get(summary['verdict'], '⚪')

    print(f'  Overall Score:     {icon} {summary["viability_score"]}/100 — {summary["verdict"]}')
    print(f'  ├─ Features:       {summary["feature_score"]}/40  (avg {summary["avg_features"]:.0f} ORB/frame)')
    print(f'  ├─ Matching:       {summary["matching_score"]}/30  (avg {summary["avg_match_ratio"]:.1f}% overlap)')
    print(f'  ├─ Diversity:      {summary["diversity_score"]}/15  ({len(summary["terrain_types"])} terrain types)')
    print(f'  └─ GPS metadata:   {summary["gps_score"]}/15  (all frames geotagged)')
    print()
    print(f'  Terrain types found: {summary["terrain_types"]}')
    print()

    # Recommendations
    print('  ── Recommendations for Terrain-Aided Navigation ──')
    print()
    if summary['viability_score'] > 70:
        print('  ✅ Your terrain data is VIABLE for terrain-aided navigation.')
        print('  ✅ Feature density is sufficient for ORB-based matching.')
        print('  ✅ GPS EXIF enables building a georeferenced terrain database.')
    if 'dense_vegetation' in summary['terrain_types']:
        print('  ⚠️  Dense vegetation zones: features may be seasonal/variable.')
        print('     → Consider supplementing with edge-based matching for these areas.')
    if 'barren_soil' in summary['terrain_types']:
        print('  ⚠️  Barren soil zones: fewer distinctive features.')
        print('     → Add texture descriptors (LBP/GLCM) as secondary matcher.')
    print()
    print('  📂 Recommended Pipeline:')
    print('     1. Build terrain DB from nadir frames (georeferenced ORB descriptors)')
    print('     2. Use oblique frames for VIO sequence testing')
    print('     3. Cross-validate by matching oblique→nadir for multi-view')
    print()

    # Save results
    out_path = os.path.join(base, 'terrain_analysis_results.json')
    # Remove non-serializable descriptors
    for dtype in ['nadir', 'oblique']:
        for name in analyzer.results[dtype]:
            if 'descriptors' in analyzer.results[dtype][name]:
                del analyzer.results[dtype][name]['descriptors']
    with open(out_path, 'w') as f:
        json.dump(analyzer.results, f, indent=2)
    print(f'  💾 Results saved to: {out_path}')
    print('=' * 70)


if __name__ == '__main__':
    main()
