#!/usr/bin/env python3
"""
Citation Validator

Purpose: Verify all citations follow standards (Tier 1/2, IEEE format, no Wikipedia) per Article 19

Usage:
    python citation.py <content_file> [--min-tier1 10] [--check-urls] [--json]

Exit Codes:
    0: Citations valid
    1: Citation violations found
"""

import argparse
import json
import re
import sys
import urllib.request
from pathlib import Path
from typing import Dict, Any, List
from urllib.parse import urlparse


class CitationValidator:
    """Validates citations and sources"""

    # Tier 1 sources (preferred - academic/official)
    TIER1_DOMAINS = [
        'ieee.org', 'springer.com', 'arxiv.org', 'acm.org',
        'roboticsproceedings.org', 'docs.nvidia.com', 'mujoco.readthedocs.io',
        'docs.ros.org', 'sciencedirect.com', 'nature.com', 'science.org'
    ]

    TIER1_KEYWORDS = [
        'ieee transactions', 'icra', 'iros', 'rss', 'ijrr', 'conference',
        'proceedings', 'journal', 'symposium'
    ]

    # Tier 2 sources (acceptable - industry/education)
    TIER2_DOMAINS = [
        'nvidia.com/blog', 'openai.com', 'github.com',
        'mit.edu', 'stanford.edu', 'cmu.edu', 'berkeley.edu'
    ]

    # Excluded sources (never cite)
    EXCLUDED_SOURCES = [
        'wikipedia.org', 'wikimedia.org', 'fandom.com', 'wikia.com'
    ]

    def __init__(self, content_file: Path, min_tier1: int = 10, check_urls: bool = False):
        self.content_file = content_file
        self.content = content_file.read_text(encoding='utf-8')
        self.min_tier1 = min_tier1
        self.check_urls = check_urls

    def validate(self) -> Dict[str, Any]:
        """Run citation validation"""

        # Extract all citations
        citation_pattern = re.compile(r'\[\d+\]\s+(.+?)(?=\[\d+\]|\Z)', re.DOTALL)
        citations = citation_pattern.findall(self.content)
        citation_count = len(citations)

        # Extract all URLs
        url_pattern = re.compile(r'https?://([^\s\)]+)')
        urls = url_pattern.findall(self.content)

        # Classify sources by tier
        tier1_count = 0
        tier2_count = 0
        excluded_sources_found = []

        for url in urls:
            url_lower = url.lower()

            # Check for excluded sources first
            if any(exc in url_lower for exc in self.EXCLUDED_SOURCES):
                excluded_sources_found.append(url)
                continue

            # Check Tier 1
            if any(domain in url_lower for domain in self.TIER1_DOMAINS):
                tier1_count += 1
            # Check Tier 2
            elif any(domain in url_lower for domain in self.TIER2_DOMAINS):
                tier2_count += 1

        # Check for Tier 1 keywords in citations (for non-URL citations)
        for citation in citations:
            citation_lower = citation.lower()
            if any(keyword in citation_lower for keyword in self.TIER1_KEYWORDS):
                if not any(url in citation for url in urls):  # Avoid double-counting
                    tier1_count += 1

        # Check citation format (IEEE style)
        formatting_errors = []
        ieee_pattern = re.compile(r'\[\d+\]\s+[A-Z]')  # Should start with capital letter
        for i, citation in enumerate(citations, 1):
            if not ieee_pattern.match(f'[{i}] {citation.strip()}'):
                formatting_errors.append({
                    'citation_text': citation[:100] + '...' if len(citation) > 100 else citation,
                    'issue': 'Does not match IEEE format (should start with author/title)',
                    'line_number': self._find_line_number(citation)
                })

        # Check URL accessibility (if requested)
        url_check_results = []
        if self.check_urls:
            url_check_results = self._check_url_accessibility(urls)

        # Determine overall validity
        citations_valid = (
            tier1_count >= self.min_tier1 and
            len(excluded_sources_found) == 0 and
            len(formatting_errors) == 0 and
            (not self.check_urls or all(r['accessible'] for r in url_check_results))
        )

        return {
            'citations_valid': citations_valid,
            'citation_count': citation_count,
            'tier1_count': tier1_count,
            'tier2_count': tier2_count,
            'excluded_sources_found': excluded_sources_found,
            'formatting_errors': formatting_errors,
            'url_check_results': url_check_results if self.check_urls else None,
            'min_tier1_required': self.min_tier1,
            'file': str(self.content_file)
        }

    def _find_line_number(self, text: str) -> int:
        """Find approximate line number for a text snippet"""
        lines = self.content.split('\n')
        for i, line in enumerate(lines, 1):
            if text[:50] in line:
                return i
        return 0

    def _check_url_accessibility(self, urls: List[str]) -> List[Dict[str, Any]]:
        """Check if URLs are accessible (HTTP status)"""
        results = []
        for url in urls:
            full_url = f'https://{url}' if not url.startswith('http') else url
            try:
                req = urllib.request.Request(
                    full_url,
                    headers={'User-Agent': 'Mozilla/5.0'}
                )
                response = urllib.request.urlopen(req, timeout=10)
                results.append({
                    'url': url,
                    'accessible': True,
                    'http_status': response.getcode()
                })
            except Exception as e:
                results.append({
                    'url': url,
                    'accessible': False,
                    'http_status': None,
                    'error': str(e)
                })
        return results


def main():
    parser = argparse.ArgumentParser(
        description='Validate citations and sources'
    )
    parser.add_argument('content_file', type=Path, help='Path to content file (markdown)')
    parser.add_argument(
        '--min-tier1',
        type=int,
        default=10,
        help='Minimum Tier 1 citations required (default: 10)'
    )
    parser.add_argument(
        '--check-urls',
        action='store_true',
        help='Check URL accessibility (HTTP requests, may be slow)'
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

    # Run validation
    validator = CitationValidator(args.content_file, args.min_tier1, args.check_urls)
    results = validator.validate()

    # Output results
    if args.json:
        print(json.dumps(results, indent=2))
    else:
        print(f"\n{'='*70}")
        print(f"Citation Validation Report: {args.content_file.name}")
        print(f"{'='*70}\n")

        if results['citations_valid']:
            print("✅ CITATIONS VALID\n")
        else:
            print("❌ CITATION VIOLATIONS FOUND\n")

        print(f"Citation Count: {results['citation_count']}")
        print(f"Tier 1 Citations: {results['tier1_count']} (required: ≥{results['min_tier1_required']})")
        print(f"Tier 2 Citations: {results['tier2_count']}\n")

        if results['tier1_count'] < results['min_tier1_required']:
            print(f"❌ Insufficient Tier 1 citations (need {results['min_tier1_required'] - results['tier1_count']} more)\n")
        else:
            print("✅ Tier 1 citation requirement met\n")

        if results['excluded_sources_found']:
            print("❌ Excluded Sources Found:")
            for source in set(results['excluded_sources_found']):
                print(f"  - {source}")
            print("\n  Replace with Tier 1/2 academic or official sources\n")

        if results['formatting_errors']:
            print("Formatting Issues:")
            for err in results['formatting_errors'][:5]:  # Show first 5
                print(f"  Line {err['line_number']}: {err['issue']}")
                print(f"    {err['citation_text']}\n")
            if len(results['formatting_errors']) > 5:
                print(f"  ... and {len(results['formatting_errors']) - 5} more\n")

        if results['url_check_results']:
            inaccessible = [r for r in results['url_check_results'] if not r['accessible']]
            if inaccessible:
                print("❌ Inaccessible URLs:")
                for r in inaccessible:
                    print(f"  - {r['url']}")
                    print(f"    Error: {r.get('error', 'Unknown')}\n")
            else:
                print("✅ All URLs accessible\n")

    # Exit with appropriate code
    sys.exit(0 if results['citations_valid'] else 1)


if __name__ == '__main__':
    main()
