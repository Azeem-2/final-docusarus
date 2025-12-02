#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Readability Validator

Purpose: Ensure text is accessible to target audience (university undergraduates) per Article 6

Usage:
    python readability.py <text_file> [--json]

Exit Codes:
    0: Readability passed
    2: Readability warnings (not blocking)
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


class ReadabilityValidator:
    """Validates readability metrics"""

    # Target ranges (university level)
    TARGET_FK_GRADE = (11, 15)  # Flesch-Kincaid Grade: 12-14 target, allow 11-15
    TARGET_READING_EASE = (45, 65)  # Flesch Reading Ease: 50-60 target, allow 45-65
    TARGET_SENTENCE_LENGTH = (12, 25)  # Words per sentence
    MAX_PASSIVE_VOICE = 15  # Percentage

    def __init__(self, text_file: Path):
        self.text_file = text_file
        raw_content = text_file.read_text(encoding='utf-8')

        # Remove markdown syntax for cleaner text analysis
        self.content = self._clean_markdown(raw_content)

    def validate(self) -> Dict[str, Any]:
        """Run readability validation"""

        # Calculate metrics
        flesch_kincaid_grade = self._calculate_fk_grade()
        flesch_reading_ease = self._calculate_reading_ease()
        avg_sentence_length = self._calculate_avg_sentence_length()
        avg_word_length = self._calculate_avg_word_length()
        passive_voice_pct = self._estimate_passive_voice()

        # Check if metrics are in acceptable ranges
        fk_in_range = self.TARGET_FK_GRADE[0] <= flesch_kincaid_grade <= self.TARGET_FK_GRADE[1]
        ease_in_range = self.TARGET_READING_EASE[0] <= flesch_reading_ease <= self.TARGET_READING_EASE[1]
        sentence_in_range = self.TARGET_SENTENCE_LENGTH[0] <= avg_sentence_length <= self.TARGET_SENTENCE_LENGTH[1]
        passive_ok = passive_voice_pct < self.MAX_PASSIVE_VOICE

        readability_passed = fk_in_range and ease_in_range and sentence_in_range and passive_ok

        # Generate recommendations
        recommendations = []
        if not fk_in_range:
            if flesch_kincaid_grade < self.TARGET_FK_GRADE[0]:
                recommendations.append(f'Text may be too simple (FK Grade {flesch_kincaid_grade:.1f}). Add more technical depth.')
            else:
                recommendations.append(f'Text may be too complex (FK Grade {flesch_kincaid_grade:.1f}). Simplify sentence structure or vocabulary.')

        if not ease_in_range:
            if flesch_reading_ease < self.TARGET_READING_EASE[0]:
                recommendations.append(f'Text is difficult to read (Ease {flesch_reading_ease:.1f}). Use shorter sentences and simpler words.')
            else:
                recommendations.append(f'Text may be too easy (Ease {flesch_reading_ease:.1f}). Consider adding more technical vocabulary.')

        if not sentence_in_range:
            if avg_sentence_length < self.TARGET_SENTENCE_LENGTH[0]:
                recommendations.append(f'Sentences too short (avg {avg_sentence_length:.1f} words). Combine related ideas.')
            else:
                recommendations.append(f'Sentences too long (avg {avg_sentence_length:.1f} words). Break into shorter sentences.')

        if not passive_ok:
            recommendations.append(f'Excessive passive voice ({passive_voice_pct:.1f}%). Use more active voice.')

        return {
            'readability_passed': readability_passed,
            'metrics': {
                'flesch_kincaid_grade': round(flesch_kincaid_grade, 1),
                'flesch_reading_ease': round(flesch_reading_ease, 1),
                'avg_sentence_length': round(avg_sentence_length, 1),
                'avg_word_length': round(avg_word_length, 1),
                'passive_voice_percentage': round(passive_voice_pct, 1)
            },
            'target_ranges': {
                'fk_grade': f'{self.TARGET_FK_GRADE[0]}-{self.TARGET_FK_GRADE[1]}',
                'reading_ease': f'{self.TARGET_READING_EASE[0]}-{self.TARGET_READING_EASE[1]}',
                'sentence_length': f'{self.TARGET_SENTENCE_LENGTH[0]}-{self.TARGET_SENTENCE_LENGTH[1]} words',
                'passive_voice': f'<{self.MAX_PASSIVE_VOICE}%'
            },
            'recommendations': recommendations,
            'file': str(self.text_file)
        }

    def _clean_markdown(self, text: str) -> str:
        """Remove markdown syntax for analysis"""
        # Remove code blocks
        text = re.sub(r'```.*?```', '', text, flags=re.DOTALL)
        # Remove inline code
        text = re.sub(r'`[^`]+`', '', text)
        # Remove headers
        text = re.sub(r'^#+\s+', '', text, flags=re.MULTILINE)
        # Remove links
        text = re.sub(r'\[([^\]]+)\]\([^\)]+\)', r'\1', text)
        # Remove emphasis
        text = re.sub(r'[*_]{1,2}([^*_]+)[*_]{1,2}', r'\1', text)
        return text

    def _count_syllables(self, word: str) -> int:
        """Estimate syllable count (simple algorithm)"""
        word = word.lower()
        vowels = 'aeiouy'
        syllable_count = 0
        previous_was_vowel = False

        for char in word:
            is_vowel = char in vowels
            if is_vowel and not previous_was_vowel:
                syllable_count += 1
            previous_was_vowel = is_vowel

        # Adjust for silent 'e'
        if word.endswith('e'):
            syllable_count -= 1

        # Ensure at least 1 syllable
        if syllable_count == 0:
            syllable_count = 1

        return syllable_count

    def _calculate_fk_grade(self) -> float:
        """Flesch-Kincaid Grade Level formula"""
        sentences = self._get_sentences()
        words = self._get_words()
        syllables = sum(self._count_syllables(w) for w in words)

        if len(sentences) == 0 or len(words) == 0:
            return 0.0

        avg_sentence_length = len(words) / len(sentences)
        avg_syllables_per_word = syllables / len(words)

        fk_grade = 0.39 * avg_sentence_length + 11.8 * avg_syllables_per_word - 15.59
        return max(0, fk_grade)

    def _calculate_reading_ease(self) -> float:
        """Flesch Reading Ease formula"""
        sentences = self._get_sentences()
        words = self._get_words()
        syllables = sum(self._count_syllables(w) for w in words)

        if len(sentences) == 0 or len(words) == 0:
            return 100.0

        avg_sentence_length = len(words) / len(sentences)
        avg_syllables_per_word = syllables / len(words)

        reading_ease = 206.835 - 1.015 * avg_sentence_length - 84.6 * avg_syllables_per_word
        return max(0, min(100, reading_ease))

    def _calculate_avg_sentence_length(self) -> float:
        """Average words per sentence"""
        sentences = self._get_sentences()
        words = self._get_words()
        return len(words) / len(sentences) if sentences else 0

    def _calculate_avg_word_length(self) -> float:
        """Average characters per word"""
        words = self._get_words()
        return sum(len(w) for w in words) / len(words) if words else 0

    def _estimate_passive_voice(self) -> float:
        """Estimate passive voice percentage (simple heuristic)"""
        # Look for "be" verb + past participle patterns
        passive_patterns = [
            r'\b(is|are|was|were|be|been|being)\s+\w+ed\b',
            r'\b(is|are|was|were|be|been|being)\s+\w+en\b'
        ]

        sentences = self._get_sentences()
        passive_count = sum(
            1 for sentence in sentences
            if any(re.search(pattern, sentence, re.IGNORECASE) for pattern in passive_patterns)
        )

        return (passive_count / len(sentences) * 100) if sentences else 0

    def _get_sentences(self) -> List[str]:
        """Split text into sentences"""
        # Simple sentence splitting
        sentences = re.split(r'[.!?]+', self.content)
        return [s.strip() for s in sentences if s.strip() and len(s.split()) > 3]

    def _get_words(self) -> List[str]:
        """Extract words from text"""
        words = re.findall(r'\b[a-zA-Z]+\b', self.content)
        return [w for w in words if len(w) > 1]  # Filter single-letter words


def main():
    parser = argparse.ArgumentParser(
        description='Validate text readability metrics'
    )
    parser.add_argument('text_file', type=Path, help='Path to text file (markdown)')
    parser.add_argument(
        '--json',
        action='store_true',
        help='Output results as JSON'
    )

    args = parser.parse_args()

    # Validate file exists
    if not args.text_file.exists():
        print(f"Error: File not found: {args.text_file}", file=sys.stderr)
        sys.exit(1)

    # Run validation
    validator = ReadabilityValidator(args.text_file)
    results = validator.validate()

    # Output results
    if args.json:
        print(json.dumps(results, indent=2))
    else:
        print(f"\n{'='*70}")
        print(f"Readability Validation Report: {args.text_file.name}")
        print(f"{'='*70}\n")

        if results['readability_passed']:
            print("✅ READABILITY PASSED - All metrics in target range\n")
        else:
            print("⚠️  READABILITY WARNINGS - Some metrics outside target range\n")

        metrics = results['metrics']
        targets = results['target_ranges']

        print("Metrics:")
        print(f"  Flesch-Kincaid Grade: {metrics['flesch_kincaid_grade']} (target: {targets['fk_grade']})")
        print(f"  Flesch Reading Ease: {metrics['flesch_reading_ease']} (target: {targets['reading_ease']})")
        print(f"  Avg Sentence Length: {metrics['avg_sentence_length']} words (target: {targets['sentence_length']})")
        print(f"  Avg Word Length: {metrics['avg_word_length']} characters")
        print(f"  Passive Voice: {metrics['passive_voice_percentage']}% (target: {targets['passive_voice']})\n")

        if results['recommendations']:
            print("Recommendations:")
            for i, rec in enumerate(results['recommendations'], 1):
                print(f"  {i}. {rec}")
            print()

    # Exit with appropriate code (warnings only, not blocking)
    sys.exit(2 if not results['readability_passed'] else 0)


if __name__ == '__main__':
    main()
