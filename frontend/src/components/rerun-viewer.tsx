'use client';

import { useEffect, useRef, useState } from 'react';
import type { WebViewer } from '@rerun-io/web-viewer';
import { cn } from '@/lib/utils';

interface RerunViewerProps {
  className?: string;
  serverUrl?: string;
}

export function RerunViewer({ className, serverUrl }: RerunViewerProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const viewerRef = useRef<WebViewer | null>(null);
  const [status, setStatus] = useState<'idle' | 'connected' | 'error'>('idle');
  const url = serverUrl ?? process.env.NEXT_PUBLIC_RERUN_WS_URL ?? 'ws://localhost:9876';

  useEffect(() => {
    let isMounted = true;

    async function setup() {
      if (!containerRef.current) return;
      try {
        const { WebViewer } = await import('@rerun-io/web-viewer');
        const viewer = new WebViewer({ rrd_server_url: url });
        viewerRef.current = viewer;
        containerRef.current.appendChild(viewer.element);
        await viewer.connect();
        if (isMounted) {
          setStatus('connected');
        }
      } catch (error) {
        console.error('Failed to initialise Rerun viewer', error);
        if (isMounted) {
          setStatus('error');
        }
      }
    }

    setup().catch(console.error);

    return () => {
      isMounted = false;
      if (viewerRef.current) {
        viewerRef.current.disconnect().catch(console.error);
        viewerRef.current.element.remove();
        viewerRef.current = null;
      }
    };
  }, [url]);

  return (
    <div className={cn('relative h-full w-full bg-black/80', className)}>
      <div ref={containerRef} className="h-full w-full" />
      {status === 'error' ? (
        <div className="absolute inset-0 flex items-center justify-center bg-black/70 text-sm text-red-400">
          无法连接到 Rerun 服务，请检查后端是否运行。
        </div>
      ) : null}
    </div>
  );
}
