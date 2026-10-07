import { describe, expect, it } from "vitest";
import { RasterRequestGeneration } from "../src/components/control/occupancyRaster";

function deferred<T>() {
  let resolve!: (value: T) => void;
  const promise = new Promise<T>((done) => { resolve = done; });
  return { promise, resolve };
}

describe("occupancy raster request ordering", () => {
  it("does not let an older raster promise replace a newer map raster", async () => {
    const generation = new RasterRequestGeneration();
    let displayedRaster = "";
    const map100 = deferred<string>();
    const token100 = generation.begin();
    const result100 = map100.promise.then((raster) => {
      if (generation.isCurrent(token100)) displayedRaster = raster;
    });

    const map101 = deferred<string>();
    const token101 = generation.begin();
    const result101 = map101.promise.then((raster) => {
      if (generation.isCurrent(token101)) displayedRaster = raster;
    });

    map101.resolve("raster-101");
    await result101;
    map100.resolve("raster-100");
    await result100;

    expect(displayedRaster).toBe("raster-101");
  });
});
