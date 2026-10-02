import { defineConfig } from 'vite';
import { svelte } from '@sveltejs/vite-plugin-svelte';
import { viteSingleFile } from 'vite-plugin-singlefile';
import path from 'path';

export default defineConfig({
  plugins: [svelte(), viteSingleFile()],
  build: {
    outDir: path.resolve(__dirname, '../firmware/arm_receiver/data'),
    emptyOutDir: false
  }
});
