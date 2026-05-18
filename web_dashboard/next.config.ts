import type { NextConfig } from "next";

// When deployed to GitHub Pages the site lives under `/<repo-name>/`, not at
// the domain root. The CI workflow injects NEXT_PUBLIC_BASE_PATH so the
// build emits correctly-prefixed asset URLs; a local `npm run dev` /
// `npm run build` without that env var keeps the historical behaviour
// (served from `/`).
const basePath = (process.env.NEXT_PUBLIC_BASE_PATH ?? "").replace(/\/$/, "");

const nextConfig: NextConfig = {
  output: "export",
  basePath: basePath || undefined,
  assetPrefix: basePath ? `${basePath}/` : undefined,
  // The static export pipeline can't run Next's image optimizer (no server
  // at request time), so disable it.
  images: { unoptimized: true },
  // Trailing slashes make every page directory-resolvable on GitHub Pages
  // (`/foo/index.html` instead of `/foo.html`).
  trailingSlash: true,
};

export default nextConfig;
