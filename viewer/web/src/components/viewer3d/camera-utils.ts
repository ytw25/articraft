import * as THREE from 'three';

export interface FitResult {
  position: THREE.Vector3;
  target: THREE.Vector3;
  near: number;
  far: number;
}

/**
 * Compute a camera position that fits the given object within the camera's field of view.
 * Returns the ideal position, target (center of bounding box), and clipping planes.
 */
export function computeFit(
  camera: THREE.PerspectiveCamera,
  object: THREE.Object3D,
): FitResult {
  const box = new THREE.Box3().setFromObject(object);
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());

  const maxDim = Math.max(size.x, size.y, size.z);
  const fov = camera.fov * (Math.PI / 180);
  // Distance needed so the object fits in view, with a small padding factor
  const distance = (maxDim / 2) / Math.tan(fov / 2) * 1.5;

  // Position camera along its current direction from center, or default diagonal
  const direction = new THREE.Vector3();
  if (camera.position.lengthSq() > 0) {
    direction.copy(camera.position).sub(center).normalize();
  }
  if (direction.lengthSq() < 0.001) {
    direction.set(1, 0.8, 1).normalize();
  }

  const position = center.clone().add(direction.multiplyScalar(distance));

  const diagonal = maxDim * Math.sqrt(3);
  const near = Math.max(0.001, distance - diagonal);
  const far = distance + diagonal * 2;

  return { position, target: center, near, far };
}

/**
 * Preserve the camera's current viewing angle and relative distance when switching
 * to a new model. Adjusts position so the new object is centered and fits in view.
 */
export function preserveViewAcrossModelSwitch(
  camera: THREE.PerspectiveCamera,
  controls: { target: THREE.Vector3; update: () => void },
  newObject: THREE.Object3D,
): void {
  const oldTarget = controls.target.clone();
  const oldOffset = camera.position.clone().sub(oldTarget);
  const oldDistance = oldOffset.length();
  const direction = oldDistance > 0.001
    ? oldOffset.normalize()
    : new THREE.Vector3(1, 0.8, 1).normalize();

  const box = new THREE.Box3().setFromObject(newObject);
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());

  const maxDim = Math.max(size.x, size.y, size.z);
  const fov = camera.fov * (Math.PI / 180);
  const fitDistance = (maxDim / 2) / Math.tan(fov / 2) * 1.5;

  // Use the old viewing direction but scale to fit the new object
  const newPosition = center.clone().add(direction.multiplyScalar(fitDistance));

  camera.position.copy(newPosition);
  controls.target.copy(center);
  controls.update();

  updateCameraClipping(camera, newObject);
}

/**
 * Update the camera's near and far clipping planes based on the object's bounding box size.
 * Ensures the full object is visible without z-fighting artifacts.
 */
export function updateCameraClipping(
  camera: THREE.PerspectiveCamera,
  object: THREE.Object3D,
): void {
  const box = new THREE.Box3().setFromObject(object);
  const size = box.getSize(new THREE.Vector3());
  const center = box.getCenter(new THREE.Vector3());

  const maxDim = Math.max(size.x, size.y, size.z);
  const distance = camera.position.distanceTo(center);
  const diagonal = maxDim * Math.sqrt(3);

  camera.near = Math.max(0.001, (distance - diagonal) * 0.5);
  camera.far = (distance + diagonal) * 2.5;
  camera.updateProjectionMatrix();
}
