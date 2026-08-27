import { readFileSync } from 'node:fs';
import { defineConfig } from 'vite';
import { svelte } from '@sveltejs/vite-plugin-svelte';

const API_ROUTES = ['/api', '/update_raw', '/update_fs_raw'];
const pkg = JSON.parse(readFileSync(new URL('./package.json', import.meta.url)));
let VERSION = process.env.ROTATOR_VERSION;
if (!VERSION) {
  try {
    VERSION = readFileSync(new URL('./build/version.txt', import.meta.url), 'utf8').trim();
  } catch {
    VERSION = pkg.version;
  }
}

function versionFile() {
  return {
    name: 'rotator-version',
    generateBundle() {
      this.emitFile({
        type: 'asset',
        fileName: 'version.json',
        source: JSON.stringify({ version: VERSION, built: new Date().toISOString() })
      });
    }
  };
}

export default defineConfig({
  define: { __ROTATOR_VERSION__: JSON.stringify(VERSION) },
  plugins: [svelte(), versionFile()],
  root: 'web',
  build: {
    outDir: '../main/data',
    emptyOutDir: true
  },
  server: {
    host: '0.0.0.0',
    proxy: Object.fromEntries(API_ROUTES.map((route) => [route, 'http://localhost:8080']))
  },
  cacheDir: process.env.HOME ? `${process.env.HOME}/.cache/rotator-vite` : '../.vite-cache',
  test: {
    include: ['../tests/**/*.test.js']
  }
});
