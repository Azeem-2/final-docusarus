/**
 * DocItem/Content Theme Swizzle (Wrap)
 *
 * Wraps the original DocItem/Content component with LessonContent
 * to provide tabbed interface for Full Lesson and AI Summary views.
 *
 * The summary is read from global data (populated by docusaurus-summaries-plugin)
 * which scans for .summary.md files at build time.
 */

import React, { useRef } from 'react';
import Content from '@theme-original/DocItem/Content';
import type ContentType from '@theme/DocItem/Content';
import type { WrapperProps } from '@docusaurus/types';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import { usePluginData } from '@docusaurus/useGlobalData';
import LessonContent from '../../../components/LessonContent';
import ReactMarkdown from 'react-markdown';

type Props = WrapperProps<typeof ContentType>;

interface SummariesPluginData {
  summaries: Record<string, string>;
}

export default function ContentWrapper(props: Props): React.ReactElement {
  const doc = useDoc();
  const contentRef = useRef<HTMLDivElement>(null);

  // Get summaries from global data (populated by docusaurus-summaries-plugin)
  let summaries: Record<string, string> = {};
  try {
    const pluginData = usePluginData('docusaurus-summaries-plugin') as SummariesPluginData | undefined;
    summaries = pluginData?.summaries || {};
  } catch {
    // Plugin might not be loaded yet or doesn't exist
    summaries = {};
  }

  // Build the lookup key from doc metadata
  const metadata = doc.metadata;
  const docId = metadata.id;

  // Debug log in development
  if (typeof window !== 'undefined' && process.env.NODE_ENV === 'development') {
    console.log('[DocItem/Content] Doc ID:', docId);
    console.log('[DocItem/Content] Available summaries:', Object.keys(summaries));
  }

  // Look up summary by doc ID (the key format matches how plugin stores them)
  const summary = summaries[docId];

  // If no summary, just render original content
  if (!summary) {
    return (
      <div ref={contentRef}>
        <Content {...props} />
      </div>
    );
  }

  // Render summary as markdown
  const summaryElement = <ReactMarkdown>{summary}</ReactMarkdown>;

  return (
    <div ref={contentRef}>
      <LessonContent summaryElement={summaryElement}>
        <Content {...props} />
      </LessonContent>
    </div>
  );
}
