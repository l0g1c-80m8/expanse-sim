'use client';

import { useEffect, useState } from 'react';

export interface ViewSettings {
  showOrbits: boolean;
  showLabels: boolean;
  showGrid: boolean;
  showStars: boolean;
  showTrajectory: boolean;
  /** Forward-prediction horizon in seconds. */
  trajectoryHorizonSec: number;
}

const DEFAULTS: ViewSettings = {
  showOrbits: true,
  showLabels: true,
  showGrid: true,
  showStars: true,
  showTrajectory: true,
  trajectoryHorizonSec: 6 * 30 * 86_400, // ~6 months
};

const KEY = 'expanse_view_settings_v1';

export function useViewSettings(): [ViewSettings, (patch: Partial<ViewSettings>) => void] {
  const [state, setState] = useState<ViewSettings>(DEFAULTS);

  useEffect(() => {
    if (typeof window === 'undefined') return;
    const raw = window.localStorage.getItem(KEY);
    if (!raw) return;
    try {
      const v = JSON.parse(raw);
      setState({ ...DEFAULTS, ...v });
    } catch {
      /* ignore */
    }
  }, []);

  const update = (patch: Partial<ViewSettings>) => {
    setState((prev) => {
      const next = { ...prev, ...patch };
      if (typeof window !== 'undefined') {
        window.localStorage.setItem(KEY, JSON.stringify(next));
      }
      return next;
    });
  };

  return [state, update];
}
