import { defineConfig } from 'vite';

export default defineConfig({
  test: {
    include: ['tests/**/*.test.ts'],
    testTimeout: 60000,
  },
} as any);
