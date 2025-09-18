import type { HTMLAttributes } from 'react';
import { cn } from '@/lib/utils';

const badgeVariantMap: Record<string, string> = {
  default: 'bg-slate-800 text-slate-200',
  success: 'bg-emerald-600/80 text-emerald-50',
  warning: 'bg-amber-500/80 text-amber-100',
  danger: 'bg-red-600/80 text-red-100'
};

export interface BadgeProps extends HTMLAttributes<HTMLSpanElement> {
  variant?: keyof typeof badgeVariantMap;
}

export function Badge({ className, variant = 'default', ...props }: BadgeProps) {
  return (
    <span
      className={cn(
        'inline-flex items-center rounded-full px-2.5 py-1 text-xs font-medium uppercase tracking-wide',
        badgeVariantMap[variant],
        className
      )}
      {...props}
    />
  );
}
