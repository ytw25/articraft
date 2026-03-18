import { useState, useCallback, useEffect, useRef } from 'react';

export interface RenderOptions {
  showEdges: boolean;
  showGrid: boolean;
  showCollisions: boolean;
  doubleSided: boolean;
  envLighting: boolean;
}

const STORAGE_KEY = 'articraft-render-options';

const DEFAULT_OPTIONS: RenderOptions = {
  showEdges: false,
  showGrid: true,
  showCollisions: false,
  doubleSided: false,
  envLighting: true,
};

export interface RenderOptionsState {
  options: RenderOptions;
  setOption: <K extends keyof RenderOptions>(key: K, value: RenderOptions[K]) => void;
  resetOptions: () => void;
}

/**
 * Read persisted render options from localStorage, falling back to defaults.
 */
function loadOptions(): RenderOptions {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (raw) {
      const parsed = JSON.parse(raw) as Partial<RenderOptions>;
      return { ...DEFAULT_OPTIONS, ...parsed };
    }
  } catch {
    // Ignore corrupt or inaccessible storage.
  }
  return { ...DEFAULT_OPTIONS };
}

/**
 * Hook for managing render option toggles.
 *
 * State is persisted to localStorage under the key 'articraft-render-options'.
 */
export function useRenderOptions(): RenderOptionsState {
  const [options, setOptions] = useState<RenderOptions>(loadOptions);
  const isInitialMount = useRef(true);

  // Persist to localStorage whenever options change (skip the initial mount).
  useEffect(() => {
    if (isInitialMount.current) {
      isInitialMount.current = false;
      return;
    }
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(options));
    } catch {
      // Storage full or unavailable -- silently ignore.
    }
  }, [options]);

  const setOption = useCallback(
    <K extends keyof RenderOptions>(key: K, value: RenderOptions[K]) => {
      setOptions((prev) => ({ ...prev, [key]: value }));
    },
    [],
  );

  const resetOptions = useCallback(() => {
    setOptions({ ...DEFAULT_OPTIONS });
  }, []);

  return { options, setOption, resetOptions };
}
