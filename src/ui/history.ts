// Snapshot undo/redo over the serialized scene.

import type { Scene } from '../model/types';

export class History {
  private undoStack: string[] = [];
  private redoStack: string[] = [];
  constructor(private limit = 200) {}

  /** Record the state *before* a mutation. */
  push(scene: Scene): void {
    const s = JSON.stringify(scene);
    if (this.undoStack[this.undoStack.length - 1] === s) return;
    this.undoStack.push(s);
    if (this.undoStack.length > this.limit) this.undoStack.shift();
    this.redoStack = [];
  }

  undo(current: Scene): Scene | null {
    const s = this.undoStack.pop();
    if (!s) return null;
    this.redoStack.push(JSON.stringify(current));
    return JSON.parse(s);
  }

  redo(current: Scene): Scene | null {
    const s = this.redoStack.pop();
    if (!s) return null;
    this.undoStack.push(JSON.stringify(current));
    return JSON.parse(s);
  }

  get canUndo(): boolean { return this.undoStack.length > 0; }
  get canRedo(): boolean { return this.redoStack.length > 0; }
}
