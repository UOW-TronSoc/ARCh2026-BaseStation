import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import path from 'path';

export default defineConfig({
  plugins: [react()],
  server: {
    host: true, // listen on 0.0.0.0 so accessible from other devices (e.g. 10.0.0.2 → http://10.0.0.1:5173)
    port: 5173,
  },
  resolve: {
    alias: {
      components: path.resolve(__dirname, 'src/components'),
      context: path.resolve(__dirname, 'src/context'),
      pages: path.resolve(__dirname, 'src/pages'),
      assets: path.resolve(__dirname, 'src/assets'),
      styles: path.resolve(__dirname, 'src/styles'),
    },
  },
});
