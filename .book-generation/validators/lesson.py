#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lesson Validator - Validates 6-part lesson structure

Validates:
- 6-part structure (Hook, Theory, Walkthrough, Challenge, Takeaways, Learn with AI)
- Word count ranges per part
- Diagram specifications (>=4)
- Lab count (>=2)
- Safety warnings (if physical labs present)
- AI touchpoints (>=5)
- Dual-domain balance

Exit codes:
- 0: All validations passed
- 1: Critical violations (missing parts, too short)
- 2: Warnings (minor issues, still acceptable)
"""

import sys
import re
import json
from pathlib import Path
from typing import Dict, List, Tuple
import io

# Force UTF-8 output on Windows
if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')


class LessonValidator:
    """Validates lesson content against 6-part template requirements"""

    REQUIRED_PARTS = [
        "Part 1: The Hook",
        "Part 2: The Concept",  # or "Part 2: Theory"
        "Part 3: The Walkthrough",
        "Part 4: The Challenge",
        "Part 5: Key Takeaways",  # or "Part 5: Takeaways"
        "Part 6: Learn with AI"
    ]

    WORD_COUNT_RANGES = {
        "Part 1": (200, 500),       # Hook
        "Part 2": (800, 2000),      # Theory
        "Part 3": (600, 1500),      # Walkthrough
        "Part 4": (400, 3000),      # Challenge (includes lab instructions)
        "Part 5": (300, 800),       # Takeaways
        "Part 6": (200, 600)        # Learn with AI
    }

    PHYSICAL_KEYWORDS = [
        'hardware', 'sensor', 'actuator', 'motor', 'servo', 'mechanical',
        'friction', 'torque', 'force', 'real robot', 'physical robot',
        'raspberry pi', 'arduino', 'imu', 'camera', 'gripper'
    ]

    SIMULATION_KEYWORDS = [
        'simulator', 'simulation', 'digital twin', 'virtual', 'physics engine',
        'isaac sim', 'mujoco', 'gazebo', 'reinforcement learning', 'rl',
        'domain randomization', 'sim-to-real', 'isaac gym'
    ]

    SAFETY_WARNING_PATTERNS = [
        r'⚠️',
        r'WARNING',
        r'CAUTION',
        r'HAZARD',
        r'safety',
        r'Safety'
    ]

    def __init__(self, lesson_path: str):
        """
        Initialize validator

        Args:
            lesson_path: Path to lesson.md file
        """
        self.lesson_path = Path(lesson_path)
        if not self.lesson_path.exists():
            raise FileNotFoundError(f"Lesson file not found: {lesson_path}")

        self.content = self.lesson_path.read_text(encoding='utf-8')
        self.violations = []
        self.warnings = []
        self.metrics = {}

    def validate(self) -> Dict:
        """
        Run all validations

        Returns:
            Dictionary with validation results
        """
        self._check_part_structure()
        self._check_word_counts()
        self._check_diagrams()
        self._check_labs()
        self._check_safety_warnings()
        self._check_ai_touchpoints()
        self._check_dual_domain_balance()

        return {
            'compliant': len(self.violations) == 0,
            'violations': self.violations,
            'warnings': self.warnings,
            'metrics': self.metrics
        }

    def _check_part_structure(self):
        """Validate all 6 parts are present"""
        part_pattern = re.compile(r'^##\s+(Part \d+:.*?)$', re.MULTILINE)
        found_parts = part_pattern.findall(self.content)

        self.metrics['parts_found'] = len(found_parts)
        self.metrics['part_titles'] = found_parts

        if len(found_parts) < 6:
            self.violations.append({
                'check': 'Part Structure',
                'severity': 'Critical',
                'message': f'Expected 6 parts, found {len(found_parts)}',
                'found': found_parts
            })
            return

        # Check if required part types are present (flexible matching)
        required_keywords = ['Hook', 'Concept', 'Theory', 'Walkthrough', 'Challenge', 'Takeaway', 'Learn with AI']
        missing = []

        for keyword in ['Hook', 'Challenge', 'Learn with AI']:  # Strict requirements
            if not any(keyword.lower() in part.lower() for part in found_parts):
                missing.append(keyword)

        if missing:
            self.violations.append({
                'check': 'Part Structure',
                'severity': 'Critical',
                'message': f'Missing required part types: {", ".join(missing)}',
                'found': found_parts
            })

    def _check_word_counts(self):
        """Validate word counts for each part"""
        # Split content by parts
        part_pattern = re.compile(r'##\s+(Part \d+:.*?)(?=##\s+Part \d+:|$)', re.DOTALL)
        parts = part_pattern.findall(self.content)

        word_counts = {}
        for part_content in parts:
            # Extract part number
            part_num_match = re.match(r'(Part \d+)', part_content)
            if part_num_match:
                part_key = part_num_match.group(1)
                # Count words (exclude markdown syntax)
                text = re.sub(r'```.*?```', '', part_content, flags=re.DOTALL)  # Remove code blocks
                text = re.sub(r'[#*`\-]', '', text)  # Remove markdown
                words = len(text.split())
                word_counts[part_key] = words

        self.metrics['word_counts'] = word_counts
        self.metrics['total_words'] = sum(word_counts.values())

        # Check ranges
        for part_key, count in word_counts.items():
            if part_key in self.WORD_COUNT_RANGES:
                min_words, max_words = self.WORD_COUNT_RANGES[part_key]
                if count < min_words:
                    self.violations.append({
                        'check': 'Word Count',
                        'severity': 'Critical',
                        'message': f'{part_key} too short: {count} words (minimum {min_words})',
                        'actual': count,
                        'expected': f'{min_words}-{max_words}'
                    })
                elif count > max_words * 1.5:  # Allow 50% overflow for comprehensive content
                    self.warnings.append({
                        'check': 'Word Count',
                        'message': f'{part_key} very long: {count} words (recommended max {max_words})',
                        'actual': count,
                        'expected': f'{min_words}-{max_words}'
                    })

    def _check_diagrams(self):
        """Validate diagram specifications (>=4)"""
        # Look for diagram indicators (including ASCII art diagrams)
        diagram_indicators = []

        # Pattern 1: Mermaid diagrams
        diagram_indicators.extend(re.findall(r'```mermaid', self.content, re.IGNORECASE))

        # Pattern 2: Figure references
        diagram_indicators.extend(re.findall(r'\*\*Figure \d+', self.content, re.IGNORECASE))
        diagram_indicators.extend(re.findall(r'Diagram \d+:', self.content, re.IGNORECASE))

        # Pattern 3: Image markdown
        diagram_indicators.extend(re.findall(r'\!\[.*?\]\(.*?\)', self.content))

        # Pattern 4: ASCII art diagrams (look for box-drawing patterns)
        ascii_diagrams = re.findall(r'```\s*\n(?:[^\n]*(?:→|->|<-|├|└|│|─|┌|┐|└|┘){1,}[^\n]*\n){3,}```', self.content, re.MULTILINE)
        diagram_indicators.extend(ascii_diagrams)

        # Pattern 5: Tables that serve as visual comparisons
        table_patterns = re.findall(r'\|[^\n]+\|[^\n]+\|\n\|[-:\s|]+\|\n(?:\|[^\n]+\|\n){2,}', self.content)
        diagram_indicators.extend(table_patterns)

        # Pattern 6: Explicit "Visual" or "Diagram" sections
        visual_sections = re.findall(r'###\s+(?:Visual|Diagram|Figure)', self.content, re.IGNORECASE)
        diagram_indicators.extend(visual_sections)

        diagram_count = len(diagram_indicators)
        self.metrics['diagram_count'] = diagram_count
        self.metrics['diagram_types'] = {
            'mermaid': len(re.findall(r'```mermaid', self.content, re.IGNORECASE)),
            'ascii_art': len(ascii_diagrams),
            'tables': len(table_patterns),
            'images': len(re.findall(r'\!\[.*?\]\(.*?\)', self.content)),
            'labeled_figures': len(re.findall(r'\*\*Figure \d+|Diagram \d+:', self.content, re.IGNORECASE))
        }

        if diagram_count < 4:
            self.violations.append({
                'check': 'Diagrams',
                'severity': 'Critical',
                'message': f'Insufficient diagrams: {diagram_count} found (minimum 4 required)',
                'actual': diagram_count,
                'expected': '>=4'
            })

    def _check_labs(self):
        """Validate lab count (>=2)"""
        # Look for lab indicators
        lab_patterns = [
            r'Lab \d+:',
            r'Challenge \d+:',
            r'### Lab',
            r'### Challenge',
        ]

        lab_count = 0
        for pattern in lab_patterns:
            matches = re.findall(pattern, self.content, re.IGNORECASE)
            lab_count += len(matches)

        self.metrics['lab_count'] = lab_count

        if lab_count < 2:
            self.violations.append({
                'check': 'Labs',
                'severity': 'Critical',
                'message': f'Insufficient labs: {lab_count} found (minimum 2 required)',
                'actual': lab_count,
                'expected': '>=2'
            })

    def _check_safety_warnings(self):
        """Validate safety warnings if physical labs present"""
        # Check for physical lab indicators
        has_physical_lab = any(
            keyword in self.content.lower()
            for keyword in ['raspberry pi', 'arduino', 'physical robot', 'hardware', 'real robot']
        )

        if not has_physical_lab:
            self.metrics['safety_warnings'] = 'N/A (no physical labs)'
            return

        # Count safety warnings
        warning_count = 0
        for pattern in self.SAFETY_WARNING_PATTERNS:
            warning_count += len(re.findall(pattern, self.content))

        self.metrics['safety_warnings'] = warning_count

        if warning_count < 2:
            self.violations.append({
                'check': 'Safety Warnings',
                'severity': 'Critical',
                'message': f'Insufficient safety warnings for physical labs: {warning_count} found (minimum 2 required)',
                'actual': warning_count,
                'expected': '>=2'
            })

    def _check_ai_touchpoints(self):
        """Validate AI integration touchpoints (>=5)"""
        # Look for AI prompt patterns in Part 6
        part6_match = re.search(r'##\s+Part 6:.*?(?=##\s+Part \d+:|$)', self.content, re.DOTALL | re.IGNORECASE)

        if not part6_match:
            self.violations.append({
                'check': 'AI Touchpoints',
                'severity': 'Critical',
                'message': 'Part 6 (Learn with AI) not found',
                'actual': 0,
                'expected': '>=5'
            })
            return

        part6_content = part6_match.group(0)

        # Count AI prompts (look for numbered prompts or subsections)
        prompt_patterns = [
            r'\d+\.\s+\*\*',  # Numbered bold items
            r'###\s+\d+\.',    # Numbered h3 headers
            r'Prompt \d+:',
            r'\*\*Prompt:',
        ]

        touchpoint_count = 0
        for pattern in prompt_patterns:
            touchpoint_count = max(touchpoint_count, len(re.findall(pattern, part6_content)))

        self.metrics['ai_touchpoints'] = touchpoint_count

        if touchpoint_count < 5:
            self.violations.append({
                'check': 'AI Touchpoints',
                'severity': 'Critical',
                'message': f'Insufficient AI touchpoints: {touchpoint_count} found (minimum 5 required)',
                'actual': touchpoint_count,
                'expected': '>=5'
            })

    def _check_dual_domain_balance(self):
        """Validate physical + simulation balance"""
        physical_count = sum(
            self.content.lower().count(keyword.lower())
            for keyword in self.PHYSICAL_KEYWORDS
        )

        simulation_count = sum(
            self.content.lower().count(keyword.lower())
            for keyword in self.SIMULATION_KEYWORDS
        )

        if max(physical_count, simulation_count) == 0:
            balance_score = 0.0
        else:
            balance_score = min(physical_count, simulation_count) / max(physical_count, simulation_count)

        self.metrics['physical_keyword_count'] = physical_count
        self.metrics['simulation_keyword_count'] = simulation_count
        self.metrics['dual_domain_balance'] = round(balance_score, 2)

        if balance_score < 0.5:
            self.violations.append({
                'check': 'Dual-Domain Balance',
                'severity': 'Critical',
                'message': f'Poor dual-domain balance: {balance_score:.2f} (minimum 0.5 required)',
                'actual': balance_score,
                'expected': '>=0.5',
                'details': f'Physical: {physical_count} mentions, Simulation: {simulation_count} mentions'
            })
        elif balance_score < 0.7:
            self.warnings.append({
                'check': 'Dual-Domain Balance',
                'message': f'Dual-domain balance below recommended threshold: {balance_score:.2f} (recommended >=0.7)',
                'actual': balance_score,
                'expected': '>=0.7',
                'details': f'Physical: {physical_count} mentions, Simulation: {simulation_count} mentions'
            })


def main():
    """Main validation entry point"""
    if len(sys.argv) < 2:
        print("Usage: python lesson.py <lesson.md>")
        sys.exit(1)

    lesson_path = sys.argv[1]

    try:
        validator = LessonValidator(lesson_path)
        results = validator.validate()

        # Print results
        print("\n" + "="*60)
        print("LESSON VALIDATION REPORT")
        print("="*60 + "\n")

        print(f"File: {lesson_path}\n")

        # Metrics
        print("METRICS:")
        print("-" * 60)
        for key, value in results['metrics'].items():
            if isinstance(value, list):
                print(f"  {key}:")
                for item in value:
                    print(f"    - {item}")
            else:
                print(f"  {key}: {value}")
        print()

        # Violations
        if results['violations']:
            print("VIOLATIONS (Critical):")
            print("-" * 60)
            for violation in results['violations']:
                print(f"  [X] {violation['check']}: {violation['message']}")
                if 'details' in violation:
                    print(f"     Details: {violation['details']}")
            print()

        # Warnings
        if results['warnings']:
            print("WARNINGS (Non-blocking):")
            print("-" * 60)
            for warning in results['warnings']:
                print(f"  [!] {warning['check']}: {warning['message']}")
                if 'details' in warning:
                    print(f"     Details: {warning['details']}")
            print()

        # Overall result
        print("="*60)
        if results['compliant']:
            print("[PASS] VALIDATION PASSED - All requirements met")
            print("="*60 + "\n")
            sys.exit(0)
        elif not results['violations']:
            print("[WARN] VALIDATION PASSED WITH WARNINGS - Acceptable but has minor issues")
            print("="*60 + "\n")
            sys.exit(2)
        else:
            print("[FAIL] VALIDATION FAILED - Critical violations found")
            print("="*60 + "\n")
            sys.exit(1)

    except FileNotFoundError as e:
        print(f"Error: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
