export type ValidatedRosMapPair = {
  yamlFile: File;
  imageFile: File;
  width: number;
  height: number;
  resolution: number;
  origin: [number, number, number];
  totalBytes: number;
  pixels: Uint8Array;
};

export type MapImportValidation =
  | { ok: true; map: ValidatedRosMapPair }
  | { ok: false; error: string };

const MAX_YAML_BYTES = 256 * 1024;
const MAX_IMAGE_BYTES = 50 * 1024 * 1024;
const MAX_MAP_PIXELS = 20_000_000;

type PgmImage = { width: number; height: number; pixels: Uint8Array };

function decodeYamlScalar(document: string, key: string): string | null {
  if (key === "origin") {
    const origin = document.match(/^\s*origin\s*:\s*(\[[^\]]*\])\s*(?:#.*)?$/mi);
    return origin?.[1]?.trim() ?? null;
  }
  const match = document.match(new RegExp(`^\\s*${key}\\s*:\\s*(?:"([^"]*)"|'([^']*)'|([^#\\s]+))\\s*(?:#.*)?$`, "mi"));
  return match ? (match[1] ?? match[2] ?? match[3] ?? "").trim() : null;
}

function parseFinite(value: string | null, label: string): number {
  if (value === null || value.trim() === "") throw new Error(`YAML is missing ${label}.`);
  const number = Number(value);
  if (!Number.isFinite(number)) throw new Error(`YAML ${label} must be a finite number.`);
  return number;
}

function parseOrigin(document: string): [number, number, number] {
  const raw = decodeYamlScalar(document, "origin");
  if (!raw || !/^\[[^\]]+\]$/.test(raw)) throw new Error("YAML origin must contain three map coordinates.");
  const values = raw.slice(1, -1).split(",").map((value) => Number(value.trim()));
  if (values.length !== 3 || !values.every(Number.isFinite)) throw new Error("YAML origin must contain three finite map coordinates.");
  return values as [number, number, number];
}

function readToken(bytes: Uint8Array, start: number): { token: string; next: number; delimiter: number } {
  let index = start;
  while (index < bytes.length) {
    const value = bytes[index];
    if (value === 35) {
      while (index < bytes.length && bytes[index] !== 10 && bytes[index] !== 13) index += 1;
    } else if (value === 9 || value === 10 || value === 13 || value === 32) {
      index += 1;
    } else {
      break;
    }
  }
  const tokenStart = index;
  while (index < bytes.length && bytes[index] !== 9 && bytes[index] !== 10 && bytes[index] !== 13 && bytes[index] !== 32 && bytes[index] !== 35) index += 1;
  if (tokenStart === index) throw new Error("PGM header is incomplete.");
  const token = new TextDecoder("ascii").decode(bytes.subarray(tokenStart, index));
  const delimiter = bytes[index];
  if (delimiter !== 9 && delimiter !== 10 && delimiter !== 13 && delimiter !== 32 && delimiter !== 35) throw new Error("PGM header delimiter is missing.");
  return { token, next: delimiter === 35 ? index : index + 1, delimiter };
}

function pgmInteger(token: string, label: string): number {
  if (!/^\d+$/.test(token)) throw new Error(`PGM ${label} must be a positive integer.`);
  const number = Number(token);
  if (!Number.isSafeInteger(number) || number <= 0) throw new Error(`PGM ${label} must be a positive integer.`);
  return number;
}

function parsePgm(bytes: Uint8Array): PgmImage {
  let index = 0;
  const magic = readToken(bytes, index); index = magic.next;
  if (magic.token !== "P2" && magic.token !== "P5") throw new Error("Only P2 and P5 PGM images are supported by the saved-map contract.");
  const widthToken = readToken(bytes, index); index = widthToken.next;
  const heightToken = readToken(bytes, index); index = heightToken.next;
  const maxValueToken = readToken(bytes, index); index = maxValueToken.next;
  const width = pgmInteger(widthToken.token, "width");
  const height = pgmInteger(heightToken.token, "height");
  const maxValue = pgmInteger(maxValueToken.token, "maximum value");
  if (maxValue !== 255) throw new Error("This project accepts only 8-bit PGM images with maximum value 255.");
  const pixelCount = width * height;
  if (!Number.isSafeInteger(pixelCount) || pixelCount > MAX_MAP_PIXELS) throw new Error("Map image exceeds the 20-megapixel staging limit.");

  const pixels = new Uint8Array(pixelCount);
  if (magic.token === "P2") {
    const data = new TextDecoder("ascii").decode(bytes.subarray(index));
    const values = data.replace(/#[^\r\n]*/g, " ").trim().split(/\s+/).filter(Boolean);
    if (values.length !== pixelCount) throw new Error("ASCII PGM pixel count does not match its dimensions.");
    for (let pixel = 0; pixel < pixelCount; pixel += 1) {
      if (!/^\d+$/.test(values[pixel])) throw new Error("ASCII PGM contains a non-integer pixel value.");
      const value = Number(values[pixel]);
      if (value > maxValue) throw new Error("ASCII PGM pixel exceeds its declared maximum value.");
      pixels[pixel] = Math.round(value * 255 / maxValue);
    }
  } else {
    if (![9, 10, 13, 32].includes(maxValueToken.delimiter)) throw new Error("Binary PGM header must end with a whitespace raster delimiter.");
    if (maxValueToken.delimiter === 13 && bytes[index] === 10) index += 1;
    if (bytes.length - index !== pixelCount) throw new Error("Binary PGM raster length does not match its dimensions.");
    pixels.set(bytes.subarray(index, index + pixelCount));
  }
  return { width, height, pixels };
}

export async function validateRosMapFiles(files: File[]): Promise<MapImportValidation> {
  try {
    if (files.length !== 2) throw new Error("Select one .yaml metadata file and one matching .pgm image.");
    const yamlFiles = files.filter((file) => file.name.toLowerCase().endsWith(".yaml"));
    const imageFiles = files.filter((file) => file.name.toLowerCase().endsWith(".pgm"));
    if (yamlFiles.length !== 1 || imageFiles.length !== 1) throw new Error("This project accepts saved-map pairs in .yaml + .pgm format.");
    const [yamlFile] = yamlFiles;
    const [imageFile] = imageFiles;
    if (yamlFile.size <= 0 || yamlFile.size > MAX_YAML_BYTES) throw new Error("YAML file must be between 1 byte and 256 KB.");
    if (imageFile.size <= 0 || imageFile.size > MAX_IMAGE_BYTES) throw new Error("PGM image must be between 1 byte and 50 MB.");

    const document = await yamlFile.text();
    const imageReference = decodeYamlScalar(document, "image");
    if (!imageReference || imageReference.includes("/") || imageReference.includes("\\") || imageReference === "." || imageReference === "..") {
      throw new Error("YAML image must reference the selected PGM file by its file name.");
    }
    if (imageReference !== imageFile.name) throw new Error("YAML image reference must match the selected PGM file name.");
    const mode = decodeYamlScalar(document, "mode");
    if (mode?.toLowerCase() !== "trinary") throw new Error("This project exports ROS maps in trinary mode.");
    const resolution = parseFinite(decodeYamlScalar(document, "resolution"), "resolution");
    if (resolution <= 0) throw new Error("Map resolution must be greater than zero.");
    const origin = parseOrigin(document);
    const negate = parseFinite(decodeYamlScalar(document, "negate"), "negate");
    const occupiedThreshold = parseFinite(decodeYamlScalar(document, "occupied_thresh"), "occupied_thresh");
    const freeThreshold = parseFinite(decodeYamlScalar(document, "free_thresh"), "free_thresh");
    if (![0, 1].includes(negate)) throw new Error("YAML negate must be 0 or 1.");
    if (!(0 <= freeThreshold && freeThreshold < occupiedThreshold && occupiedThreshold <= 1)) {
      throw new Error("Map thresholds must satisfy 0 ≤ free_thresh < occupied_thresh ≤ 1.");
    }

    const image = parsePgm(new Uint8Array(await imageFile.arrayBuffer()));
    return {
      ok: true,
      map: { yamlFile, imageFile, width: image.width, height: image.height, resolution, origin,
        totalBytes: yamlFile.size + imageFile.size, pixels: image.pixels },
    };
  } catch (caught) {
    return { ok: false, error: caught instanceof Error ? caught.message : "Map files could not be validated." };
  }
}
