// Builds this folder's Rust core into single/<core>.wasm with installed tools
// only (offline; nothing is installed) and writes build-metadata.json, the
// source/output receipt that scripts/check.mjs verifies. A successful build
// shows that the core compiles, not that it is correct or fast.
import { spawnSync } from "node:child_process";
import { createHash } from "node:crypto";
import { existsSync, mkdirSync, readFileSync, readdirSync, writeFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const core = "ground_contact";
// The crate this build compiles, then its path dependencies.
const crates = ["ground-contact"];
const root = fileURLToPath(new URL("../", import.meta.url));
// Intermediates go under the owning repository's ignored build/.
const targetDirectory = path.join(root, "..", "build", path.basename(root), "target", "single");
const lock = JSON.parse(readFileSync(path.join(root, "toolchain.lock.json")));
function run(command, args, env = process.env) {
  const result = spawnSync(command, args, { cwd: root, env, encoding: "utf8" });
  if (result.error || result.status !== 0) throw new Error(`${command} ${args.join(" ")} failed: ${result.error?.message ?? result.stderr}`);
  return result.stdout.trim();
}
const rust = run("rustup", ["run", "stable", "rustc", "--version"]);
const cargo = run("rustup", ["run", "stable", "cargo", "--version"]);
if (!rust.startsWith(`rustc ${lock.rust} `) || !cargo.startsWith(`cargo ${lock.cargo} `)) throw new Error("Installed Rust/cargo differs from toolchain.lock.json; nothing was installed");
if (!run("rustup", ["target", "list", "--installed", "--toolchain", "stable"]).split(/\s+/).includes(lock.target)) throw new Error(`Missing installed target ${lock.target}; ask the user before running rustup target add ${lock.target} --toolchain stable. Nothing was installed`);
const binaryen = process.env.WASM_OPT ?? (existsSync("/opt/homebrew/opt/emscripten/libexec/binaryen/bin/wasm-opt") ? "/opt/homebrew/opt/emscripten/libexec/binaryen/bin/wasm-opt" : "wasm-opt");
const optimizer = run(binaryen, ["--version"]);
if (!new RegExp(`\\b${lock.binaryen}\\b`).test(optimizer)) throw new Error("Binaryen differs from toolchain.lock.json");
const flags = `-C target-feature=+simd128 -C link-arg=--max-memory=${lock.maximumMemoryBytes}`;
const sha = file => createHash("sha256").update(readFileSync(path.join(root, file))).digest("hex");
function files(directory) {
  return readdirSync(path.join(root, directory), { withFileTypes: true }).flatMap(entry => {
    const file = path.posix.join(directory, entry.name);
    return entry.isDirectory() ? files(file) : [file];
  }).sort();
}

const args = ["+stable", "build", "-p", crates[0], "--release", "--locked", "--offline", "--target", lock.target];
run("cargo", args, { ...process.env, RUSTFLAGS: flags, CARGO_TARGET_DIR: targetDirectory });
const input = path.join(targetDirectory, lock.target, "release", `${crates[0].replaceAll("-", "_")}.wasm`);
const output = `single/${core}.wasm`;
mkdirSync(path.join(root, "single"), { recursive: true });
const optimize = ["-O3", "--enable-simd", "--strip-debug"];
run(binaryen, [input, ...optimize, "-o", path.join(root, output)]);
const bytes = readFileSync(path.join(root, output));
if (!WebAssembly.validate(bytes)) throw new Error(`${output} does not validate in this Node runtime`);
if (bytes.length > lock.maximumBinaryBytes) throw new Error(`${output} exceeds the declared binary size limit`);
// The wrapper and pins beside the binary ship with it, so they are inputs too.
const beside = readdirSync(root, { withFileTypes: true })
  .filter(entry => entry.isFile() && /\.(js|ts|json)$/.test(entry.name) && !entry.name.endsWith("build-metadata.json")).map(entry => entry.name);
const inputs = [...new Set(["Cargo.toml", "Cargo.lock", "rust-toolchain.toml", "toolchain.lock.json", "scripts/build.mjs", ...crates.flatMap(crate => files(`crates/${crate}`)), ...beside])].sort();
const metadata = {
  schema: 1, core, abiVersion: 2, rust, cargo, optimizer, node: process.versions.node, target: lock.target,
  build: { command: ["cargo", ...args], rustflags: flags, optimize: ["wasm-opt", ...optimize] },
  inputs: Object.fromEntries(inputs.map(file => [file, sha(file)])),
  outputs: { [output]: { sha256: sha(output), bytes: bytes.length } },
  threaded: false, functionality: "build receipt only; parity evidence comes from scripts/check.mjs", performance: "unmeasured", power: "unmeasured",
};
writeFileSync(path.join(root, "build-metadata.json"), JSON.stringify(metadata, null, 2) + "\n");
console.log(JSON.stringify({ core, output, ...metadata.outputs[output] }));
