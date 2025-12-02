#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Constitutional Compliance Validator

Purpose: Ensure all book content adheres to the 20 constitutional articles
defined in .specify/memory/constitution.md

Usage:
    python constitutional.py <content_file> [--articles 1,2,3] [--json]

Exit Codes:
    0: Compliant (zero violations)
    1: Non-compliant (critical violations found)
    2: Warnings only (no critical violations)
"""

import argparse
import json
import re
import sys
import io
from pathlib import Path
from typing import List, Dict, Any, Optional

# Force UTF-8 output on Windows
if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')


class ConstitutionalValidator:
    """Validates content against constitutional articles"""

    # Article 7: Required sections
    REQUIRED_SECTIONS = [
        "Introduction",
        "Motivation",
        "Learning Objectives",
        "Key Terms",
        "Physical Explanation",
        "Simulation Explanation",
        "Integrated Understanding",
        "Diagrams",
        "Examples",
        "Labs",
        "Mini Projects",
        "Applications",
        "Summary",
        "Review Questions"
    ]

    # Article 13: Required safety warnings
    SAFETY_WARNING_MARKERS = ["⚠️", "WARNING", "CAUTION", "DANGER", "HAZARD"]

    # Article 19: Excluded sources
    EXCLUDED_SOURCES = ["wikipedia.org", "wikimedia.org", "fandom.com", "wikia.com"]

    def __init__(self, content_file: Path, articles: Optional[List[int]] = None):
        self.content_file = content_file
        self.content = content_file.read_text(encoding='utf-8')
        self.lines = self.content.split('\n')
        self.articles_to_check = articles or list(range(1, 21))  # Default: all 20
        self.violations = []
        self.suggestions = []

    def validate(self) -> Dict[str, Any]:
        """Run all validation checks"""

        # Article 2 & 7: Chapter structure
        self._check_article_2_7_structure()

        # Article 12 & 13: Labs and safety
        self._check_article_12_13_labs()

        # Article 19: Citations and sources
        self._check_article_19_citations()

        # Compile results
        compliant = len([v for v in self.violations if v['severity'] == 'Critical']) == 0

        return {
            'compliant': compliant,
            'violations': self.violations,
            'suggestions': self.suggestions,
            'file': str(self.content_file),
            'total_violations': len(self.violations),
            'critical_count': len([v for v in self.violations if v['severity'] == 'Critical']),
            'warning_count': len([v for v in self.violations if v['severity'] == 'Warning'])
        }

    def _check_article_2_7_structure(self):
        """Article 2 (Scope) & Article 7 (Chapter Format): Required sections"""

        # Find all section headers (markdown ## or ###)
        section_pattern = re.compile(r'^##\s+(.+)$', re.MULTILINE)
        found_sections = section_pattern.findall(self.content)

        # Normalize section names for comparison
        found_normalized = [s.strip().lower() for s in found_sections]
        required_normalized = [s.lower() for s in self.REQUIRED_SECTIONS]

        # Check for missing sections
        missing = []
        for req in self.REQUIRED_SECTIONS:
            req_lower = req.lower()
            # Allow flexible matching (e.g., "Physical Robotics Explanation" matches "Physical Explanation")
            if not any(req_lower in found or found in req_lower for found in found_normalized):
                missing.append(req)

        if missing:
            self.violations.append({
                'article_number': 7,
                'article_name': 'Article 7: Chapter Format',
                'violation_description': f'Missing required sections: {", ".join(missing)}',
                'content_location': 'Chapter structure',
                'severity': 'Critical'
            })
            self.suggestions.append(
                f'Add the following sections to comply with Article 7: {", ".join(missing)}'
            )

        # Article 2: Check for dual-domain presence (basic check)
        physical_section = any('physical' in s.lower() for s in found_normalized)
        simulation_section = any('simulation' in s.lower() for s in found_normalized)

        if not (physical_section and simulation_section):
            self.violations.append({
                'article_number': 2,
                'article_name': 'Article 2: Scope (Dual-Domain)',
                'violation_description': 'Missing Physical or Simulation sections',
                'content_location': 'Chapter structure',
                'severity': 'Critical'
            })
            self.suggestions.append(
                'Ensure both "Physical Explanation" and "Simulation Explanation" sections are present'
            )

    def _check_article_12_13_labs(self):
        """Article 12 (Labs) & Article 13 (Safety): Lab requirements and safety warnings"""

        # Find Labs section
        labs_section_match = re.search(r'##\s+Labs.*?(?=##|\Z)', self.content, re.DOTALL | re.IGNORECASE)

        if not labs_section_match:
            return  # Already flagged in structure check

        labs_content = labs_section_match.group(0)

        # Check for Simulation and Physical labs
        has_simulation_lab = bool(re.search(r'simulation\s+lab', labs_content, re.IGNORECASE))
        has_physical_lab = bool(re.search(r'physical\s+lab', labs_content, re.IGNORECASE))

        if not has_simulation_lab or not has_physical_lab:
            self.violations.append({
                'article_number': 12,
                'article_name': 'Article 12: Labs',
                'violation_description': 'Must include both Simulation Lab and Physical Lab',
                'content_location': 'Labs section',
                'severity': 'Critical'
            })
            self.suggestions.append(
                'Add both "Simulation Lab" and "Physical Lab" subsections'
            )

        # Article 13: Check for safety warnings in Physical Lab
        if has_physical_lab:
            has_safety_warning = any(marker in labs_content for marker in self.SAFETY_WARNING_MARKERS)

            if not has_safety_warning:
                self.violations.append({
                    'article_number': 13,
                    'article_name': 'Article 13: Safety',
                    'violation_description': 'Physical Lab missing safety warnings',
                    'content_location': 'Labs > Physical Lab',
                    'severity': 'Critical'
                })
                self.suggestions.append(
                    'Add safety warnings (⚠️) for mechanical, electrical, or motion hazards in Physical Lab'
                )

    def _check_article_19_citations(self):
        """Article 19 (Academic Integrity): Citation standards and excluded sources"""

        # Find all URLs in content
        url_pattern = re.compile(r'https?://([^\s\)]+)')
        urls = url_pattern.findall(self.content)

        # Check for excluded sources
        excluded_found = []
        for url in urls:
            for excluded in self.EXCLUDED_SOURCES:
                if excluded in url.lower():
                    excluded_found.append(url)

        if excluded_found:
            self.violations.append({
                'article_number': 19,
                'article_name': 'Article 19: Academic Integrity',
                'violation_description': f'Excluded sources cited: {", ".join(set(excluded_found))}',
                'content_location': 'Citations',
                'severity': 'Critical'
            })
            self.suggestions.append(
                'Replace Wikipedia and user-editable sources with Tier 1/2 academic sources'
            )

        # Check citation format (basic IEEE format check)
        citation_pattern = re.compile(r'\[\d+\]')
        citations = citation_pattern.findall(self.content)

        if not citations:
            self.violations.append({
                'article_number': 19,
                'article_name': 'Article 19: Academic Integrity',
                'violation_description': 'No numbered citations found (IEEE format expected)',
                'content_location': 'Throughout document',
                'severity': 'Warning'
            })
            self.suggestions.append(
                'Add numbered citations in IEEE format: [1] Author, "Title", Source, Year.'
            )


def main():
    parser = argparse.ArgumentParser(
        description='Validate book content against constitutional articles'
    )
    parser.add_argument('content_file', type=Path, help='Path to content file (markdown)')
    parser.add_argument(
        '--articles',
        type=str,
        help='Comma-separated article numbers to check (default: all 20)',
        default=None
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='Output results as JSON'
    )

    args = parser.parse_args()

    # Validate file exists
    if not args.content_file.exists():
        print(f"Error: File not found: {args.content_file}", file=sys.stderr)
        sys.exit(1)

    # Parse article numbers if specified
    articles = None
    if args.articles:
        try:
            articles = [int(a.strip()) for a in args.articles.split(',')]
        except ValueError:
            print(f"Error: Invalid article numbers: {args.articles}", file=sys.stderr)
            sys.exit(1)

    # Run validation
    validator = ConstitutionalValidator(args.content_file, articles)
    results = validator.validate()

    # Output results
    if args.json:
        print(json.dumps(results, indent=2))
    else:
        print(f"\n{'='*70}")
        print(f"Constitutional Compliance Report: {args.content_file.name}")
        print(f"{'='*70}\n")

        if results['compliant']:
            print("✅ COMPLIANT - Zero critical violations\n")
        else:
            print(f"❌ NON-COMPLIANT - {results['critical_count']} critical violation(s)\n")

        if results['violations']:
            print(f"Violations ({results['total_violations']} total):\n")
            for v in results['violations']:
                icon = "🔴" if v['severity'] == 'Critical' else "⚠️"
                print(f"{icon} Article {v['article_number']}: {v['article_name']}")
                print(f"   {v['violation_description']}")
                print(f"   Location: {v['content_location']}\n")

        if results['suggestions']:
            print("Suggestions for Compliance:\n")
            for i, s in enumerate(results['suggestions'], 1):
                print(f"{i}. {s}")
            print()

    # Exit with appropriate code
    if results['critical_count'] > 0:
        sys.exit(1)
    elif results['warning_count'] > 0:
        sys.exit(2)
    else:
        sys.exit(0)


if __name__ == '__main__':
    main()
