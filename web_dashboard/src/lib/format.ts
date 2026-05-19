/**
 * Small formatting helpers used in multiple dashboard panels.
 */

/** Render seconds as "1.2 s" / "3m 42s" / "5h 11m" / "12d 3h" / "1.4 yr". */
export function formatSimTime(s: number): string {
  if (!Number.isFinite(s)) return '—';
  if (s < 60) return `${s.toFixed(1)} s`;
  if (s < 3600) return `${Math.floor(s / 60)}m ${Math.floor(s % 60)}s`;
  if (s < 86_400) {
    const h = Math.floor(s / 3600);
    const m = Math.floor((s % 3600) / 60);
    return `${h}h ${m}m`;
  }
  if (s < 365 * 86_400) {
    const d = Math.floor(s / 86_400);
    const h = Math.floor((s % 86_400) / 3600);
    return `${d}d ${h}h`;
  }
  const y = s / (365.25 * 86_400);
  return `${y.toFixed(2)} yr`;
}
