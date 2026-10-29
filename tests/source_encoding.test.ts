import { readdirSync, readFileSync, statSync } from "node:fs";
import { join, resolve } from "node:path";
import { describe, expect, it } from "vitest";

function sourceFiles(directory: string): string[] {
  return readdirSync(directory).flatMap((name) => {
    const path = join(directory, name);
    return statSync(path).isDirectory() ? sourceFiles(path)
      : /\.(?:ts|tsx)$/.test(name) ? [path] : [];
  });
}

describe("frontend source encoding", () => {
  it("contains no known UTF-8 mojibake or replacement characters", () => {
    const files = sourceFiles(resolve(process.cwd(), "src"));
    const patterns = [
      /\u00c2\u00b7/u,
      /\u0102\u2014/u,
      /m\u00c2\u00b2/u,
      /\u00e2\u2020/u,
      /\u00ef\u00bf\u00bd/u,
      /\ufffd/u,
    ];
    const failures = files.flatMap((file) => {
      const content = readFileSync(file, "utf8");
      return patterns.some((pattern) => pattern.test(content)) ? [file] : [];
    });
    expect(failures).toEqual([]);
  });
});
