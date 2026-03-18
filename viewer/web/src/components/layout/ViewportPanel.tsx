import { type JSX } from "react";
import type * as THREE from "three";

import { useViewer } from "@/lib/viewer-context";
import { SceneCanvas } from "@/components/viewer3d/SceneCanvas";
import type { RenderOptions } from "@/components/viewer3d/useRenderOptions";
import type { UrdfSpec } from "@/components/viewer3d/urdf-parser";

interface ViewportPanelProps {
  renderOptions: RenderOptions;
  onUrdfSpecChange: (spec: UrdfSpec | null, jointNodes: Map<string, THREE.Object3D> | null) => void;
}

export function ViewportPanel({ renderOptions, onUrdfSpecChange }: ViewportPanelProps): JSX.Element {
  const { selectedRecordId } = useViewer();

  // Map from useRenderOptions format to RenderOptionsPanel format for SceneCanvas
  const sceneRenderOptions = {
    showEdges: renderOptions.showEdges,
    showGrid: renderOptions.showGrid,
    showCollisions: renderOptions.showCollisions,
    doubleSided: renderOptions.doubleSided,
    environmentLighting: renderOptions.envLighting,
  };

  return (
    <section className="relative h-full w-full min-w-0 overflow-hidden bg-[#ebebeb]">
      <SceneCanvas
        recordId={selectedRecordId}
        renderOptions={sceneRenderOptions}
        onUrdfSpecChange={onUrdfSpecChange}
      />
    </section>
  );
}
