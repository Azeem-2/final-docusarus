#!/usr/bin/env python3
"""
Safety Validator

Purpose: Ensure all physical labs have proper hazard warnings per Article 13

Usage:
    python safety.py <lab_file> [--json]

Exit Codes:
    0: Safety compliant
    1: Safety violations found
    2: Safety review required (high-risk procedures detected)
"""

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Dict, Any, List, Tuple


class SafetyValidator:
    """Validates safety warnings in physical labs"""

    # Hazard detection keywords
    HAZARDS = {
        'mechanical': {
            'keywords': ['gear', 'pinch', 'rotating', 'sharp', 'blade', 'cut', 'crush'],
            'required_warnings': ['⚠️ PINCH HAZARD', '⚠️ SHARP EDGES', '⚠️ ROTATING PARTS']
        },
        'electrical': {
            'keywords': ['voltage', 'current', 'battery', 'lipo', 'wire', 'short circuit', 'power supply'],
            'required_warnings': ['⚠️ ELECTRICAL', '⚠️ LITHIUM POLYMER BATTERY']
        },
        'motion': {
            'keywords': ['motor', 'servo', 'movement', 'collision', 'unexpected', 'runaway'],
            'required_warnings': ['⚠️ ROBOT CAN MOVE', '⚠️ MOVEMENT ZONE', '⚠️ COLLISION HAZARD']
        }
    }

    # Emergency stop keywords
    E_STOP_KEYWORDS = ['emergency stop', 'e-stop', 'kill switch', 'power disconnect']

    # High-risk procedures requiring professional review
    HIGH_RISK_PROCEDURES = [
        ('lipo battery charging', 'LiPo battery charging/handling'),
        ('ac voltage', 'AC voltage (>50V AC)'),
        (r'\d+a', 'High current (>5A)'),  # Regex for current
        ('drill', 'Powered tools (drills, saws)'),
        ('saw', 'Powered tools (drills, saws)'),
        ('solvent', 'Hazardous materials (solvents, adhesives)'),
        ('adhesive', 'Hazardous materials (solvents, adhesives)')
    ]

    def __init__(self, lab_file: Path):
        self.lab_file = lab_file
        self.content = lab_file.read_text(encoding='utf-8')
        self.content_lower = self.content.lower()

    def validate(self) -> Dict[str, Any]:
        """Run safety validation"""

        # Find all warning markers in content
        warning_markers = re.findall(r'⚠️|WARNING|CAUTION|DANGER|HAZARD', self.content, re.IGNORECASE)
        warnings_present = len(warning_markers) > 0

        # Detect hazards and check for corresponding warnings
        hazard_types_covered = []
        issues = []

        for hazard_type, hazard_info in self.HAZARDS.items():
            hazard_detected = any(kw in self.content_lower for kw in hazard_info['keywords'])

            if hazard_detected:
                # Check if any required warning is present
                warning_found = any(
                    warning.lower() in self.content.lower()
                    for warning in hazard_info['required_warnings']
                )

                if warning_found:
                    hazard_types_covered.append(hazard_type.capitalize())
                else:
                    issues.append({
                        'hazard_description': f'{hazard_type.capitalize()} hazard detected',
                        'warning_missing': True,
                        'suggested_warning': f'Add one of: {", ".join(hazard_info["required_warnings"])}'
                    })

        # Check for emergency stop mention (required for motorized systems)
        motor_keywords = ['motor', 'servo', 'actuator']
        has_motorized_system = any(kw in self.content_lower for kw in motor_keywords)
        emergency_stop_mentioned = any(kw in self.content_lower for kw in self.E_STOP_KEYWORDS)

        if has_motorized_system and not emergency_stop_mentioned:
            issues.append({
                'hazard_description': 'Motorized system without emergency stop',
                'warning_missing': True,
                'suggested_warning': 'Add emergency stop procedure (e-stop button or power disconnect)'
            })

        # Detect high-risk procedures
        high_risk_detected = []
        for pattern, description in self.HIGH_RISK_PROCEDURES:
            if re.search(pattern, self.content_lower):
                high_risk_detected.append(description)

        safety_review_required = len(high_risk_detected) > 0

        # Determine overall compliance
        safety_compliant = warnings_present and len(issues) == 0

        return {
            'safety_compliant': safety_compliant,
            'warnings_present': warnings_present,
            'warnings_count': len(warning_markers),
            'hazard_types_covered': hazard_types_covered,
            'emergency_stop_mentioned': emergency_stop_mentioned,
            'safety_review_required': safety_review_required,
            'high_risk_procedures': high_risk_detected,
            'issues': issues,
            'file': str(self.lab_file)
        }


def main():
    parser = argparse.ArgumentParser(
        description='Validate safety warnings in physical labs'
    )
    parser.add_argument('lab_file', type=Path, help='Path to lab file (markdown)')
    parser.add_argument(
        '--json',
        action='store_true',
        help='Output results as JSON'
    )

    args = parser.parse_args()

    # Validate file exists
    if not args.lab_file.exists():
        print(f"Error: File not found: {args.lab_file}", file=sys.stderr)
        sys.exit(1)

    # Run validation
    validator = SafetyValidator(args.lab_file)
    results = validator.validate()

    # Output results
    if args.json:
        print(json.dumps(results, indent=2))
    else:
        print(f"\n{'='*70}")
        print(f"Safety Validation Report: {args.lab_file.name}")
        print(f"{'='*70}\n")

        if results['safety_compliant']:
            print("✅ SAFETY COMPLIANT\n")
        else:
            print("❌ SAFETY VIOLATIONS FOUND\n")

        print(f"Warnings Present: {'Yes' if results['warnings_present'] else 'No'} ({results['warnings_count']} total)")
        print(f"Emergency Stop Mentioned: {'Yes' if results['emergency_stop_mentioned'] else 'No'}")
        print(f"Hazard Types Covered: {', '.join(results['hazard_types_covered']) if results['hazard_types_covered'] else 'None'}\n")

        if results['issues']:
            print("Safety Issues:")
            for i, issue in enumerate(results['issues'], 1):
                print(f"  {i}. {issue['hazard_description']}")
                print(f"     Suggestion: {issue['suggested_warning']}\n")

        if results['safety_review_required']:
            print("⚠️  SAFETY REVIEW REQUIRED ⚠️")
            print("High-risk procedures detected:")
            for proc in results['high_risk_procedures']:
                print(f"  - {proc}")
            print("\nA certified safety professional must review this lab before publication.\n")

    # Exit with appropriate code
    if not results['safety_compliant']:
        sys.exit(1)
    elif results['safety_review_required']:
        sys.exit(2)
    else:
        sys.exit(0)


if __name__ == '__main__':
    main()
