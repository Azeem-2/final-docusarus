#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dual-Domain Validator

Purpose: Enforce 100% dual-domain integration (physical + simulation) per Article 2

Usage:
    python dual_domain.py <chapter_file> [--threshold 0.7] [--json]

Exit Codes:
    0: Dual-domain present (all checks pass)
    1: Dual-domain absent (fails validation)
"""

import argparse
import json
import re
import sys
import io
from pathlib import Path
from typing import Dict, Any, List

# Force UTF-8 output on Windows
if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')


class DualDomainValidator:
    """Validates dual-domain integration (physical + simulation)"""

    # Physical domain keywords
    PHYSICAL_KEYWORDS = [
        'hardware', 'sensor', 'actuator', 'motor', 'servo', 'gear', 'encoder', 'imu', 'lidar', 'camera',
        'voltage', 'current', 'power', 'battery', 'lipo', 'circuit', 'wiring', 'ground',
        'mechanical', 'friction', 'torque', 'force', 'mass', 'inertia', 'backlash',
        'real robot', 'physical robot', 'embodied', 'real-world', 'deployment'
    ]

    # Simulation domain keywords
    SIMULATION_KEYWORDS = [
        'simulator', 'simulation', 'digital twin', 'virtual', 'physics engine',
        'isaac sim', 'mujoco', 'gazebo', 'webots', 'unity robotics', 'pybullet',
        'reinforcement learning', 'rl', 'imitation learning', 'policy', 'training',
        'domain randomization', 'sim-to-real', 'transfer', 'reality gap', 'fidelity'
    ]

    def __init__(self, chapter_file: Path, threshold: float = 0.7):
        self.chapter_file = chapter_file
        self.content = chapter_file.read_text(encoding='utf-8').lower()
        self.threshold = threshold

    def validate(self) -> Dict[str, Any]:
        """Run dual-domain validation"""

        # Check for required sections
        physical_section_exists = self._check_section_exists('physical')
        simulation_section_exists = self._check_section_exists('simulation')
        integrated_section_exists = self._check_section_exists('integrated', alternatives=['comparison', 'sim-to-real'])

        # Count keyword occurrences
        physical_count = self._count_keywords(self.PHYSICAL_KEYWORDS)
        simulation_count = self._count_keywords(self.SIMULATION_KEYWORDS)

        # Calculate balance score
        if physical_count == 0 and simulation_count == 0:
            balance_score = 0.0
        elif physical_count == 0 or simulation_count == 0:
            balance_score = 0.0
        else:
            balance_score = min(physical_count, simulation_count) / max(physical_count, simulation_count)

        # Determine if dual-domain is present
        dual_domain_present = (
            physical_section_exists and
            simulation_section_exists and
            integrated_section_exists and
            balance_score >= self.threshold
        )

        # Generate recommendations
        recommendations = []
        if not physical_section_exists:
            recommendations.append('Add "Physical Explanation" section with hardware details')
        if not simulation_section_exists:
            recommendations.append('Add "Simulation Explanation" section with virtual environment details')
        if not integrated_section_exists:
            recommendations.append('Add "Integrated Understanding" section comparing physical and simulation')
        if balance_score < self.threshold:
            if physical_count < simulation_count:
                recommendations.append(f'Increase physical domain coverage (currently {physical_count} vs {simulation_count} simulation keywords)')
            else:
                recommendations.append(f'Increase simulation domain coverage (currently {simulation_count} vs {physical_count} physical keywords)')

        return {
            'dual_domain_present': dual_domain_present,
            'physical_section_exists': physical_section_exists,
            'simulation_section_exists': simulation_section_exists,
            'integrated_section_exists': integrated_section_exists,
            'physical_keywords_count': physical_count,
            'simulation_keywords_count': simulation_count,
            'balance_score': round(balance_score, 2),
            'threshold': self.threshold,
            'recommendations': recommendations,
            'file': str(self.chapter_file)
        }

    def _check_section_exists(self, keyword: str, alternatives: List[str] = None) -> bool:
        """Check if a section with the given keyword exists"""
        alternatives = alternatives or []
        all_keywords = [keyword] + alternatives

        # Look for section headers containing the keyword
        section_pattern = re.compile(r'^##\s+.*(' + '|'.join(all_keywords) + r').*$', re.MULTILINE | re.IGNORECASE)
        match = section_pattern.search(self.content)

        if match:
            return True

        # Alternative check: Look for keyword concentration (≥10 keywords)
        keyword_count = sum(1 for kw in all_keywords if kw in self.content)
        return keyword_count >= 10

    def _count_keywords(self, keywords: List[str]) -> int:
        """Count occurrences of keywords in content"""
        count = 0
        for keyword in keywords:
            # Use word boundaries to avoid partial matches
            pattern = re.compile(r'\b' + re.escape(keyword) + r'\b', re.IGNORECASE)
            count += len(pattern.findall(self.content))
        return count


def main():
    parser = argparse.ArgumentParser(
        description='Validate dual-domain integration (physical + simulation)'
    )
    parser.add_argument('chapter_file', type=Path, help='Path to chapter file (markdown)')
    parser.add_argument(
        '--threshold',
        type=float,
        default=0.7,
        help='Balance score threshold (0.0-1.0, default: 0.7)'
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='Output results as JSON'
    )

    args = parser.parse_args()

    # Validate file exists
    if not args.chapter_file.exists():
        print(f"Error: File not found: {args.chapter_file}", file=sys.stderr)
        sys.exit(1)

    # Validate threshold
    if not 0.0 <= args.threshold <= 1.0:
        print(f"Error: Threshold must be between 0.0 and 1.0", file=sys.stderr)
        sys.exit(1)

    # Run validation
    validator = DualDomainValidator(args.chapter_file, args.threshold)
    results = validator.validate()

    # Output results
    if args.json:
        print(json.dumps(results, indent=2))
    else:
        print(f"\n{'='*70}")
        print(f"Dual-Domain Validation Report: {args.chapter_file.name}")
        print(f"{'='*70}\n")

        if results['dual_domain_present']:
            print("✅ DUAL-DOMAIN PRESENT - All checks passed\n")
        else:
            print("❌ DUAL-DOMAIN ABSENT - Validation failed\n")

        print("Section Checks:")
        print(f"  Physical Section: {'✓' if results['physical_section_exists'] else '✗'}")
        print(f"  Simulation Section: {'✓' if results['simulation_section_exists'] else '✗'}")
        print(f"  Integrated Section: {'✓' if results['integrated_section_exists'] else '✗'}\n")

        print("Keyword Analysis:")
        print(f"  Physical Keywords: {results['physical_keywords_count']}")
        print(f"  Simulation Keywords: {results['simulation_keywords_count']}")
        print(f"  Balance Score: {results['balance_score']} (threshold: {results['threshold']})\n")

        if results['balance_score'] >= results['threshold']:
            print("✅ Balance score meets threshold\n")
        else:
            print(f"❌ Balance score below threshold (need ≥{results['threshold']})\n")

        if results['recommendations']:
            print("Recommendations:")
            for i, rec in enumerate(results['recommendations'], 1):
                print(f"  {i}. {rec}")
            print()

    # Exit with appropriate code
    sys.exit(0 if results['dual_domain_present'] else 1)


if __name__ == '__main__':
    main()
