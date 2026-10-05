// Browser-independent check of pixel mapping under responsive canvas scaling.
// Uses only Node's built-ins. Browser rendering remains a separate smoke check.
"use strict";
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");
const assert = require("node:assert/strict");
const html = fs.readFileSync(path.join(__dirname, "calibration_picker.html"), "utf8");
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
new vm.Script(script, {filename: "calibration_picker.html"});
const start = script.indexOf("function imageCoordinates(");
const end = script.indexOf("function status(", start);
const context = vm.createContext({Math});
vm.runInContext("const roundPixel=value=>Math.round(value*10)/10;\n" + script.slice(start, end), context);
const point = (...args) => JSON.parse(JSON.stringify(context.imageCoordinates(...args)));
const rect = {left: 100, top: 60, right: 740, bottom: 540, width: 640, height: 480};
assert.deepEqual(point(420, 300, rect, 320, 240), {u: 160, v: 120});
assert.deepEqual(point(260, 180, rect, 320, 240), {u: 80, v: 60});
assert.deepEqual(point(740, 540, rect, 320, 240), {u: 319, v: 239});
assert.equal(point(99, 100, rect, 320, 240), null);
assert.equal(point(100, 100, {...rect, width: 0}, 320, 240), null);
const downscaled = {left: 12, top: 20, right: 172, bottom: 140, width: 160, height: 120};
assert.deepEqual(point(52, 80, downscaled, 320, 240), {u: 80, v: 120});
console.log("Picker: JavaScript syntax and 6 scaled-coordinate cases passed (no browser rendering claim).");
