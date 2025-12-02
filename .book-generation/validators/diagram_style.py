#!/usr/bin/env python3
"""
Diagram Style Validator

Purpose: Ensure all diagrams follow unified visual theme per Article 10

Usage:
    python diagram_style.py <diagram_file_or_content> [--json]

Exit Codes:
    0: Style compliant
    2: Style warnings (requires manual review)
"""

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Dict, Any, List


class DiagramStyleValidator:
    """Validates diagram style consistency"""

    # Standard color palette (as defined in style guide)
    STANDARD_COLORS = {
        '#0066CC': 'Physical Robotics (Blue)',
        '#00CC66': 'Simulation (Green)',
        '#FF9900': 'AI/ML (Orange)',
        '#666666': 'General/Framework (Gray)',
        '#000000': 'Black (text/borders)',
        '#FFFFFF': 'White (background)'
    }

    # Notation standards
    NOTATION_RULES = {
        'solid_arrow': 'Data flow',
        'dashed_arrow': 'Control flow',
        'rectangle': 'Component',
        'oval': 'Process',
        'diamond': 'Decision'
    }

    # Minimum font size (in points)
    MIN_FONT_SIZE = 12

    def __init__(self, diagram_file: Path):
        self.diagram_file = diagram_file
        self.content = diagram_file.read_text(encoding='utf-8')
        self.is_mermaid = 'mermaid' in self.content.lower() or diagram_file.suffix == '.mmd'

    def validate(self) -> Dict[str, Any]:
        """Run diagram style validation"""

        issues = []

        # Check 1: Color palette compliance
        color_issue = self._check_color_palette()
        if color_issue:
            issues.append(color_issue)

        # Check 2: Notation compliance (for Mermaid diagrams)
        if self.is_mermaid:
            notation_issue = self._check_notation()
            if notation_issue:
                issues.append(notation_issue)

        # Check 3: Labels presence
        label_issue = self._check_labels()
        if label_issue:
            issues.append(label_issue)

        # Check 4: Font size (basic check for Mermaid)
        if self.is_mermaid:
            font_issue = self._check_font_size()
            if font_issue:
                issues.append(font_issue)

        # Check 5: Caption presence
        caption_issue = self._check_caption()
        if caption_issue:
            issues.append(caption_issue)

        style_compliant = len(issues) == 0

        return {
            'style_compliant': style_compliant,
            'issues': issues,
            'is_mermaid': self.is_mermaid,
            'file': str(self.diagram_file)
        }

    def _check_color_palette(self) -> Dict[str, Any] | None:
        """Check if colors match standard palette"""
        # Extract hex colors from content
        color_pattern = re.compile(r'#[0-9A-Fa-f]{6}\b')
        found_colors = set(color_pattern.findall(self.content.upper()))

        # Filter out standard colors
        non_standard_colors = [c for c in found_colors if c not in self.STANDARD_COLORS]

        if non_standard_colors:
            return {
                'check_name': 'Color Palette',
                'passed': False,
                'details': f'Non-standard colors found: {", ".join(non_standard_colors)}. Use standard palette.'
            }

        return None

    def _check_notation(self) -> Dict[str, Any] | None:
        """Check if Mermaid notation follows standards"""
        # Basic check: Look for arrows and verify they use standard notation
        has_arrows = '-->' in self.content or '--->' in self.content or '--' in self.content

        if not has_arrows:
            return {
                'check_name': 'Notation',
                'passed': False,
                'details': 'No arrows found. Diagrams should show relationships with arrows (solid for data, dashed for control).'
            }

        # Check for proper arrow usage (basic heuristic)
        solid_arrows = len(re.findall(r'-->', self.content))
        dashed_arrows = len(re.findall(r'-\.->', self.content))

        if solid_arrows == 0 and dashed_arrows == 0:
            return {
                'check_name': 'Notation',
                'passed': False,
                'details': 'Use proper arrow notation: --> for data flow, -.-> for control flow'
            }

        return None

    def _check_labels(self) -> Dict[str, Any] | None:
        """Check if diagram components are labeled"""
        if self.is_mermaid:
            # Look for nodes with labels (format: id[Label] or id(Label))
            label_pattern = re.compile(r'[A-Za-z0-9_]+[\[\(][^\]\)]+[\]\)]')
            labels = label_pattern.findall(self.content)

            if not labels:
                return {
                    'check_name': 'Labels',
                    'passed': False,
                    'details': 'No labeled components found. All diagram components must have clear text labels.'
                }

        # For non-Mermaid, check for any text content
        else:
            if len(self.content.strip()) < 50:
                return {
                    'check_name': 'Labels',
                    'passed': False,
                    'details': 'Diagram appears to have minimal text. Ensure all components are labeled.'
                }

        return None

    def _check_font_size(self) -> Dict[str, Any] | None:
        """Check font size specification (for Mermaid)"""
        # Look for fontSize in theme config
        fontsize_pattern = re.compile(r'fontSize[:\s]+["\']?(\d+)(?:pt)?["\']?', re.IGNORECASE)
        match = fontsize_pattern.search(self.content)

        if match:
            fontsize = int(match.group(1))
            if fontsize < self.MIN_FONT_SIZE:
                return {
                    'check_name': 'Font Size',
                    'passed': False,
                    'details': f'Font size {fontsize}pt is below minimum {self.MIN_FONT_SIZE}pt. Increase for readability.'
                }

        return None

    def _check_caption(self) -> Dict[str, Any] | None:
        """Check if diagram has caption"""
        # Look for Figure caption (markdown format)
        caption_pattern = re.compile(r'^\*\*Figure\s+\d+:\s+.+\*\*', re.MULTILINE | re.IGNORECASE)
        match = caption_pattern.search(self.content)

        if not match:
            return {
                'check_name': 'Caption',
                'passed': False,
                'details': 'No caption found. Add caption in format: **Figure N: Description**'
            }

        return None


def main():
    parser = argparse.ArgumentParser(
        description='Validate diagram style consistency'
    )
    parser.add_argument('diagram_file', type=Path, help='Path to diagram file (Mermaid, SVG, or markdown with diagram)')
    parser.add_argument(
        '--json',
        action='store_true',
        help='Output results as JSON'
    )

    args = parser.parse_args()

    # Validate file exists
    if not args.diagram_file.exists():
        print(f"Error: File not found: {args.diagram_file}", file=sys.stderr)
        sys.exit(1)

    # Run validation
    validator = DiagramStyleValidator(args.diagram_file)
    results = validator.validate()

    # Output results
    if args.json:
        print(json.dumps(results, indent=2))
    else:
        print(f"\n{'='*70}")
        print(f"Diagram Style Validation Report: {args.diagram_file.name}")
        print(f"{'='*70}\n")

        diagram_type = "Mermaid" if results['is_mermaid'] else "Other"
        print(f"Diagram Type: {diagram_type}\n")

        if results['style_compliant']:
            print("✅ STYLE COMPLIANT - All checks passed\n")
        else:
            print("⚠️  STYLE ISSUES - Manual review recommended\n")

        if results['issues']:
            print("Style Issues:")
            for issue in results['issues']:
                icon = "✓" if issue['passed'] else "✗"
                print(f"  {icon} {issue['check_name']}")
                if not issue['passed']:
                    print(f"     {issue['details']}\n")
        else:
            print("No issues found. All style checks passed.\n")

        print("Note: This validator performs automated checks only.")
        print("Manual review by technical illustrator recommended for complex diagrams.\n")

    # Exit with warning code if issues found (not blocking, but requires review)
    sys.exit(2 if not results['style_compliant'] else 0)


if __name__ == '__main__':
    main()
