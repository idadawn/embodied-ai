import { clsx, type ClassValue } from 'clsx';
import { twMerge } from 'tailwind-merge';

export function cn(...inputs: ClassValue[]) {
  return twMerge(clsx(inputs));
}

export const MCP_API_BASE = process.env.NEXT_PUBLIC_MCP_API_URL ?? 'http://localhost:8080/api';
