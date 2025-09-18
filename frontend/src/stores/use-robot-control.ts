'use client';

import { create } from 'zustand';
import { MCP_API_BASE } from '@/lib/utils';

type CommandStatus = 'idle' | 'sending' | 'success' | 'error';

interface CommandHistoryItem {
  id: string;
  command: string;
  timestamp: number;
  status: CommandStatus;
  message?: string;
}

interface RobotControlState {
  currentCommand: string;
  status: CommandStatus;
  history: CommandHistoryItem[];
  setCurrentCommand: (value: string) => void;
  sendCommand: (command?: string) => Promise<void>;
  setStatus: (status: CommandStatus) => void;
}

export const useRobotControl = create<RobotControlState>((set, get) => ({
  currentCommand: '拿起可乐',
  status: 'idle',
  history: [],
  setCurrentCommand: (value: string) => set({ currentCommand: value }),
  setStatus: (status: CommandStatus) => set({ status }),
  sendCommand: async (command) => {
    const state = get();
    const payload = (command ?? state.currentCommand).trim();
    if (!payload) {
      return;
    }

    const entry: CommandHistoryItem = {
      id: crypto.randomUUID(),
      command: payload,
      timestamp: Date.now(),
      status: 'sending'
    };

    set({ status: 'sending', history: [entry, ...state.history] });

    try {
      const response = await fetch(`${MCP_API_BASE}/commands`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({ command: payload })
      });

      if (!response.ok) {
        throw new Error(`后端返回错误码 ${response.status}`);
      }

      const result = await response.json();
      set((prev) => ({
        status: 'success',
        history: [{ ...entry, status: 'success', message: result?.message }, ...prev.history.slice(1)]
      }));
    } catch (error) {
      console.error('Failed to send command', error);
      const message = error instanceof Error ? error.message : '未知错误';
      set((prev) => ({
        status: 'error',
        history: [{ ...entry, status: 'error', message }, ...prev.history.slice(1)]
      }));
    }
  }
}));
