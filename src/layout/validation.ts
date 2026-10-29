/** Rectangle collision applies only to physical objects; map geometry has dedicated validators. */
export type PhysicalRect = { kind: string; id: string; box: { x: number; z: number; w: number; h: number } };

const nonPhysicalKinds = new Set(["floor", "aisle", "navigation-tag", "zone", "walkway", "camera", "sensor", "location", "spawn"]);

export function validatePhysicalObjectOverlaps(entries: PhysicalRect[]): string[] {
  const physical = entries.filter((entry) => !nonPhysicalKinds.has(entry.kind));
  const errors: string[] = [];
  for (let i = 0; i < physical.length; i += 1) for (let j = i + 1; j < physical.length; j += 1) {
    const a = physical[i], b = physical[j];
    if (a.box.x < b.box.x + b.box.w && a.box.x + a.box.w > b.box.x && a.box.z < b.box.z + b.box.h && a.box.z + a.box.h > b.box.z) errors.push(`${a.id} overlaps ${b.id}`);
  }
  return errors;
}
