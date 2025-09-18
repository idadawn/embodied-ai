import type { Metadata } from 'next';
import { Inter } from 'next/font/google';
import type { ReactNode } from 'react';

import '@/styles/globals.css';

const inter = Inter({ subsets: ['latin'], variable: '--font-sans' });

export const metadata: Metadata = {
  title: 'EmbodiedAI Control Center',
  description: 'EmbodiedAI MVP dashboard with Rerun-powered 3D digital twin.'
};

export default function RootLayout({ children }: { children: ReactNode }) {
  return (
    <html lang="zh-CN">
      <body className={`${inter.variable} min-h-screen bg-slate-950 text-slate-50 antialiased`}>
        {children}
      </body>
    </html>
  );
}
