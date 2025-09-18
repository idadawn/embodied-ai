'use client';

import { useMemo } from 'react';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { Badge } from '@/components/ui/badge';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card';
import { RerunViewer } from '@/components/rerun-viewer';
import { useRobotControl } from '@/stores/use-robot-control';

const statusMap: Record<string, { label: string; variant: 'default' | 'success' | 'warning' | 'danger' }> = {
  idle: { label: '空闲', variant: 'default' },
  sending: { label: '发送中', variant: 'warning' },
  success: { label: '成功', variant: 'success' },
  error: { label: '错误', variant: 'danger' }
};

export default function DashboardPage() {
  const { currentCommand, setCurrentCommand, sendCommand, status, history } = useRobotControl();
  const statusInfo = useMemo(() => statusMap[status] ?? statusMap.idle, [status]);

  return (
    <div className="flex h-screen flex-col">
      <header className="border-b border-slate-800 bg-slate-950/70 px-6 py-4">
        <div className="flex flex-col gap-4 lg:flex-row lg:items-center lg:justify-between">
          <form
            className="flex w-full flex-1 items-center gap-3"
            onSubmit={(event) => {
              event.preventDefault();
              void sendCommand();
            }}
          >
            <Input
              placeholder="输入指令，例如：拿起可乐"
              value={currentCommand}
              onChange={(event) => setCurrentCommand(event.target.value)}
            />
            <Button type="submit" disabled={status === 'sending'}>
              {status === 'sending' ? '发送中…' : '发送指令'}
            </Button>
          </form>
          <Badge variant={statusInfo.variant}>任务状态：{statusInfo.label}</Badge>
        </div>
      </header>

      <main className="flex flex-1 flex-col gap-4 p-6 lg:flex-row">
        <section className="relative flex-1 overflow-hidden rounded-xl border border-slate-800 bg-slate-900">
          <RerunViewer className="h-full" />
          <div className="pointer-events-none absolute left-0 top-0 z-10 bg-gradient-to-b from-black/50 to-transparent p-4">
            <h2 className="text-lg font-semibold text-slate-100">Rerun 数字孪生</h2>
            <p className="text-sm text-slate-400">实时查看机械手、目标物体与执行轨迹。</p>
          </div>
        </section>

        <aside className="flex w-full flex-col gap-4 lg:max-w-sm">
          <Card>
            <CardHeader>
              <CardTitle>执行概览</CardTitle>
              <CardDescription>模型推理延迟、抓取成功率等关键指标。</CardDescription>
            </CardHeader>
            <CardContent className="space-y-3 text-sm">
              <div className="flex items-center justify-between">
                <span className="text-slate-400">最新推理延迟</span>
                <span className="font-medium text-slate-100">--</span>
              </div>
              <div className="flex items-center justify-between">
                <span className="text-slate-400">Rerun 延迟</span>
                <span className="font-medium text-slate-100">--</span>
              </div>
              <div className="flex items-center justify-between">
                <span className="text-slate-400">抓取成功率 (10 次)</span>
                <span className="font-medium text-slate-100">--</span>
              </div>
            </CardContent>
          </Card>

          <Card>
            <CardHeader>
              <CardTitle>指令历史</CardTitle>
              <CardDescription>查看最近的任务请求及反馈。</CardDescription>
            </CardHeader>
            <CardContent className="space-y-2">
              {history.length === 0 ? (
                <p className="text-sm text-slate-400">暂无历史记录，发送第一条指令试试吧。</p>
              ) : (
                <ul className="space-y-2">
                  {history.map((item) => {
                    const info = statusMap[item.status] ?? statusMap.idle;
                    return (
                      <li
                        key={item.id}
                        className="rounded-lg border border-slate-800 bg-slate-900/80 px-3 py-2 text-sm"
                      >
                        <div className="flex items-center justify-between">
                          <span className="font-medium text-slate-100">{item.command}</span>
                          <Badge variant={info.variant}>{info.label}</Badge>
                        </div>
                        <p className="mt-1 text-xs text-slate-400">
                          {new Date(item.timestamp).toLocaleTimeString()} {item.message ? `· ${item.message}` : ''}
                        </p>
                      </li>
                    );
                  })}
                </ul>
              )}
            </CardContent>
          </Card>
        </aside>
      </main>
    </div>
  );
}
