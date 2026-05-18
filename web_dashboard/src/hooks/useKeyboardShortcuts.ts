'use client';

import { useEffect } from 'react';

export interface ShortcutHandlers {
  onTogglePause?: () => void;
  onWarpUp?: () => void;
  onWarpDown?: () => void;
  onCutEngine?: () => void;
  onFocusShip?: () => void;
  onReset?: () => void;
}

/**
 * Global keyboard shortcuts for the dashboard. Handlers are wired to keys:
 *
 * | key       | action            |
 * |-----------|-------------------|
 * | Space     | toggle pause      |
 * | `[`       | warp down         |
 * | `]`       | warp up           |
 * | `x` / `X` | cut engine        |
 * | `f` / `F` | focus on ship     |
 * | `r` / `R` | reset             |
 *
 * Ignores events when a form input has focus, so the WS URL field still
 * works normally.
 */
export function useKeyboardShortcuts(h: ShortcutHandlers) {
  useEffect(() => {
    const handler = (e: KeyboardEvent) => {
      const target = e.target as HTMLElement | null;
      if (target) {
        const tag = target.tagName;
        if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;
        if (target.isContentEditable) return;
      }
      if (e.metaKey || e.ctrlKey || e.altKey) return;
      switch (e.key) {
        case ' ':
        case 'Space':
          e.preventDefault();
          h.onTogglePause?.();
          break;
        case '[':
          h.onWarpDown?.();
          break;
        case ']':
          h.onWarpUp?.();
          break;
        case 'x':
        case 'X':
          h.onCutEngine?.();
          break;
        case 'f':
        case 'F':
          h.onFocusShip?.();
          break;
        case 'r':
        case 'R':
          h.onReset?.();
          break;
        default:
          break;
      }
    };
    window.addEventListener('keydown', handler);
    return () => window.removeEventListener('keydown', handler);
  }, [h]);
}
