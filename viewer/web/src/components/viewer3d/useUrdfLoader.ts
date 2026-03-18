import { useState, useEffect, useRef } from 'react';
import * as THREE from 'three';
import { parseUrdf, rewriteAbsoluteMeshFilenames } from './urdf-parser';
import type { UrdfSpec } from './urdf-parser';
import { buildRobotSceneGraph } from './scene-graph-builder';
import { computeFit } from './camera-utils';
import { positionGroundHelpers } from './lighting';

/** Sentinel name attached to the robot group so we can find and remove it later. */
const ROBOT_GROUP_NAME = '__articraft_robot__';

export interface UrdfLoaderState {
  urdfSpec: UrdfSpec | null;
  jointNodes: Map<string, THREE.Object3D> | null;
  loading: boolean;
  error: string | null;
}

/**
 * Hook that fetches and loads a URDF for a given record.
 *
 * When recordId changes the hook:
 * 1. Fetches the URDF from /api/records/{recordId}/files/model.urdf
 * 2. Parses with parseUrdf, rewrites absolute mesh filenames
 * 3. Builds the scene graph with buildRobotSceneGraph
 * 4. Removes the previous robot group from the scene and adds the new one
 * 5. Fits the camera to the new model with computeFit
 * 6. Positions the grid and axis helpers underneath the model
 */
export function useUrdfLoader(
  recordId: string | null,
  scene: THREE.Scene | null,
  camera: THREE.PerspectiveCamera | null,
  controls: { target: THREE.Vector3; update: () => void } | null,
  gridGroup: THREE.Group | null,
  axisGroup: THREE.Group | null,
): UrdfLoaderState {
  const [urdfSpec, setUrdfSpec] = useState<UrdfSpec | null>(null);
  const [jointNodes, setJointNodes] = useState<Map<string, THREE.Object3D> | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Track the current robot group so we can remove it when swapping models.
  const robotGroupRef = useRef<THREE.Group | null>(null);

  useEffect(() => {
    // Reset when no record is selected.
    if (!recordId || !scene) {
      setUrdfSpec(null);
      setJointNodes(null);
      setError(null);
      return;
    }

    let cancelled = false;

    const load = async () => {
      setLoading(true);
      setError(null);

      try {
        // 1. Fetch URDF text.
        const response = await fetch(`/api/records/${recordId}/files/model.urdf`);
        if (!response.ok) {
          throw new Error(`Failed to fetch URDF: ${response.status} ${response.statusText}`);
        }
        const urdfXml = await response.text();
        if (cancelled) return;

        // 2. Parse and rewrite mesh filenames.
        const rawSpec = parseUrdf(urdfXml);
        const spec = rewriteAbsoluteMeshFilenames(rawSpec);

        // 3. Build scene graph.
        // Always include collision geometry in the scene graph so the render toggle
        // can reveal it without rebuilding the whole robot.
        const { root, jointNodes: joints } = buildRobotSceneGraph(spec, { showCollisions: true });
        if (cancelled) return;

        // 4. Swap robot group in scene.
        if (robotGroupRef.current) {
          scene.remove(robotGroupRef.current);
          disposeGroup(robotGroupRef.current);
        }

        const robotGroup = new THREE.Group();
        robotGroup.name = ROBOT_GROUP_NAME;
        robotGroup.add(root);
        scene.add(robotGroup);
        robotGroupRef.current = robotGroup;

        // 5. Fit camera.
        if (camera && controls) {
          const fit = computeFit(camera, robotGroup);
          camera.position.copy(fit.position);
          controls.target.copy(fit.target);
          controls.update();
        }

        // 6. Position grid and axis under the model.
        if (gridGroup && axisGroup) {
          positionGroundHelpers(gridGroup, axisGroup, robotGroup);
        }

        if (!cancelled) {
          setUrdfSpec(spec);
          setJointNodes(joints);
        }
      } catch (err) {
        if (!cancelled) {
          setError(err instanceof Error ? err.message : 'Unknown error loading URDF');
          setUrdfSpec(null);
          setJointNodes(null);
        }
      } finally {
        if (!cancelled) {
          setLoading(false);
        }
      }
    };

    void load();

    return () => {
      cancelled = true;
    };
  }, [recordId, scene, camera, controls, gridGroup, axisGroup]);

  return { urdfSpec, jointNodes, loading, error };
}

/**
 * Recursively dispose all geometry and materials within a group.
 */
function disposeGroup(group: THREE.Object3D): void {
  group.traverse((object) => {
    if (object instanceof THREE.Mesh) {
      object.geometry?.dispose();
      const mat = object.material;
      if (Array.isArray(mat)) {
        mat.forEach((m) => m.dispose());
      } else {
        mat?.dispose();
      }
    }
  });
}
