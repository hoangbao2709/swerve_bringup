import { layout, useStore } from "../../state/store";
import { ShelfInventoryPanel } from "./ShelfInventoryPanel";

/** Shelf inventory content hosted by the window manager. */
export function ShelfInventoryWindow({ shelfId }: { shelfId: string }) {
  const shelf = layout.racks.find((rack) => rack.id === shelfId);
  const layoutRevision = useStore((state) => state.layoutRevision);

  if (!shelf) {
    return <div className="shelf-inventory-card">Shelf {shelfId} is not available.</div>;
  }

  const load = Math.max(0, Math.min(8, Number(shelf.current_load ?? 0)));
  const percentLabel = `${Math.round((load / 8) * 100)}`;

  return (
    <ShelfInventoryPanel
      key={`${shelfId}:${layoutRevision}`}
      rackId={shelfId}
      zone={shelf.zone ?? ""}
      floor={shelf.floor ?? 1}
      position={shelf.position}
      load={load}
      percentLabel={percentLabel}
    />
  );
}
