# sm-view web client

Read-only WebGL2 viewer plus a one-line `script.run` console (Q8). No runtime dependencies.

```
npm ci            # dev dependencies only (typescript, esbuild, playwright-core)
npm test          # tsc, then node --test (codec, camera, pick, raster, rpc, batch)
npm run typecheck # tsc --noEmit --strict
npm run build     # bundle to dist/ and write dist.sha256 (commit both)
SM_VIEWER_E2E=1 npm run e2e   # headless Chromium; skips with a reason if none is found
```

Open `dist/index.html` through any static server: `?fixture=hexbox` replays a recorded session
offline, `?ws=ws://host:port` talks to a live server (default `ws://<page host>`).
See `../DESIGN.md`.
