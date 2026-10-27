// Preset scenes (filled in at the presets milestone).

import type { Scene } from './model/types';

export interface Preset {
  id: string;
  title: string;
  build: () => Scene;
}

export const PRESETS: Preset[] = [];
