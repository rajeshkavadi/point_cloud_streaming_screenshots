// Decoder unit tests — run in-browser via tests.html, or under Node:
//   node web/js/decoder_tests.js
(function (root, factory) {
  if (typeof module === "object" && module.exports) {
    module.exports = factory(require("./pc2.js"));
  } else {
    root.DecoderTests = factory(root.PC2);
  }
})(typeof self !== "undefined" ? self : this, function (PC2) {
  "use strict";

  const { INT8, UINT8, INT16, UINT16, INT32, UINT32, FLOAT32, FLOAT64 } = {
    INT8: 1, UINT8: 2, INT16: 3, UINT16: 4,
    INT32: 5, UINT32: 6, FLOAT32: 7, FLOAT64: 8,
  };
  const DT_SIZE = { 1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8 };

  function encode(points, layout, pointStep, bigEndian) {
    const buf = new ArrayBuffer(pointStep * points.length);
    const view = new DataView(buf);
    const le = !bigEndian;
    points.forEach((pt, i) => {
      layout.forEach(([name, offset, datatype], k) => {
        const base = i * pointStep + offset;
        const v = pt[name] !== undefined ? pt[name] : pt[k];
        switch (datatype) {
          case INT8: view.setInt8(base, v); break;
          case UINT8: view.setUint8(base, v); break;
          case INT16: view.setInt16(base, v, le); break;
          case UINT16: view.setUint16(base, v, le); break;
          case INT32: view.setInt32(base, v, le); break;
          case UINT32: view.setUint32(base, v, le); break;
          case FLOAT32: view.setFloat32(base, v, le); break;
          case FLOAT64: view.setFloat64(base, v, le); break;
        }
      });
    });
    return new Uint8Array(buf);
  }

  function toRosbridgeMsg(bytes, layout, pointStep, opts) {
    opts = opts || {};
    let b64 = "";
    if (typeof btoa !== "undefined") {
      for (let i = 0; i < bytes.length; i++) b64 += String.fromCharCode(bytes[i]);
      b64 = btoa(b64);
    } else {
      b64 = Buffer.from(bytes).toString("base64");
    }
    return {
      fields: layout.map(([name, offset, datatype]) => ({ name, offset, datatype, count: 1 })),
      data: b64,
      point_step: pointStep,
      row_step: pointStep * (opts.width !== undefined ? opts.width : bytes.length / pointStep),
      width: opts.width !== undefined ? opts.width : bytes.length / pointStep,
      height: opts.height || 1,
      is_dense: !!opts.is_dense,
      is_bigendian: !!opts.bigEndian,
    };
  }

  const XYZ = [["x", 0, FLOAT32], ["y", 4, FLOAT32], ["z", 8, FLOAT32]];

  const tests = [];
  function test(name, fn) { tests.push({ name, fn }); }

  test("contiguous float32 XYZ", () => {
    const pts = [{ x: 1, y: 2, z: 3 }, { x: -4.5, y: 0, z: 10.25 }];
    const msg = toRosbridgeMsg(encode(pts, XYZ, 12), XYZ, 12);
    const r = PC2.decodePointCloud2(msg);
    assertEqual(r.count, 2);
    assertClose(Array.from(r.positions), [1, 2, 3, -4.5, 0, 10.25]);
    assertEqual(r.intensity, null);
  });

  test("non-contiguous fields (padded layout)", () => {
    const layout = [["x", 0, FLOAT32], ["tag", 4, UINT16], ["pad", 6, UINT16],
                    ["y", 8, FLOAT32], ["z", 16, FLOAT32]];
    const pts = [{ x: 1, tag: 7, pad: 0, y: 2, z: 3 },
                 { x: 4, tag: 9, pad: 0, y: 5, z: 6 }];
    const msg = toRosbridgeMsg(encode(pts, layout, 24), layout, 24);
    const r = PC2.decodePointCloud2(msg);
    assertEqual(r.count, 2);
    assertClose(Array.from(r.positions), [1, 2, 3, 4, 5, 6]);
  });

  test("float64 XYZ", () => {
    const layout = [["x", 0, FLOAT64], ["y", 8, FLOAT64], ["z", 16, FLOAT64]];
    const msg = toRosbridgeMsg(encode([{ x: 1.5, y: -2.5, z: 3.5 }], layout, 24), layout, 24);
    const r = PC2.decodePointCloud2(msg);
    assertClose(Array.from(r.positions), [1.5, -2.5, 3.5]);
  });

  test("int16 XYZ (scaled integer lidar)", () => {
    const layout = [["x", 0, INT16], ["y", 2, INT16], ["z", 4, INT16]];
    const msg = toRosbridgeMsg(encode([{ x: 100, y: -200, z: 300 }], layout, 6), layout, 6);
    const r = PC2.decodePointCloud2(msg);
    assertClose(Array.from(r.positions), [100, -200, 300]);
  });

  test("big-endian message", () => {
    const msg = toRosbridgeMsg(encode([{ x: 1, y: 2, z: 3 }], XYZ, 12, true), XYZ, 12, { bigEndian: true });
    const r = PC2.decodePointCloud2(msg);
    assertClose(Array.from(r.positions), [1, 2, 3]);
  });

  test("intensity extraction", () => {
    const layout = XYZ.concat([["intensity", 12, FLOAT32]]);
    const msg = toRosbridgeMsg(encode([{ x: 1, y: 2, z: 3, intensity: 0.75 }], layout, 16), layout, 16);
    const r = PC2.decodePointCloud2(msg);
    assertEqual(r.count, 1);
    assertClose(Array.from(r.intensity), [0.75]);
  });

  test("invalid (NaN/Inf) points skipped", () => {
    const NaN_ = NaN, Inf = Infinity;
    const pts = [{ x: 1, y: 2, z: 3 }, { x: NaN_, y: 2, z: 3 },
                 { x: 1, y: Inf, z: 3 }, { x: 1, y: 2, z: -Inf },
                 { x: 4, y: 5, z: 6 }];
    const msg = toRosbridgeMsg(encode(pts, XYZ, 12), XYZ, 12);
    const r = PC2.decodePointCloud2(msg);
    assertEqual(r.count, 2);
    assertEqual(r.skipped, 3);
    assertClose(Array.from(r.positions), [1, 2, 3, 4, 5, 6]);
  });

  test("empty cloud", () => {
    const msg = toRosbridgeMsg(new Uint8Array(0), XYZ, 12, { width: 0 });
    const r = PC2.decodePointCloud2(msg);
    assertEqual(r.count, 0);
  });

  test("maxPoints stride subsample", () => {
    const n = 1000;
    const pts = [];
    for (let i = 0; i < n; i++) pts.push({ x: i, y: 0, z: 0 });
    const msg = toRosbridgeMsg(encode(pts, XYZ, 12), XYZ, 12);
    const r = PC2.decodePointCloud2(msg, { maxPoints: 100 });
    assertEqual(r.count, 100);
    assertEqual(r.total, 1000);
  });

  test("multi-row cloud with row_step padding", () => {
    // 2 rows x 2 cols with 8 bytes padding per row
    const layout = XYZ;
    const pointStep = 12, rowStep = 12 * 2 + 8;
    const buf = new ArrayBuffer(rowStep * 2);
    const view = new DataView(buf);
    const vals = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12];
    for (let i = 0; i < 4; i++) {
      const base = Math.floor(i / 2) * rowStep + (i % 2) * pointStep;
      view.setFloat32(base, vals[i * 3], true);
      view.setFloat32(base + 4, vals[i * 3 + 1], true);
      view.setFloat32(base + 8, vals[i * 3 + 2], true);
    }
    const msg = toRosbridgeMsg(new Uint8Array(buf), layout, pointStep, { width: 2, height: 2 });
    const r = PC2.decodePointCloud2(msg);
    assertEqual(r.count, 4);
    assertClose(Array.from(r.positions), vals);
  });

  test("missing xyz fields throws", () => {
    const layout = [["foo", 0, FLOAT32]];
    const msg = toRosbridgeMsg(encode([{ foo: 1 }], layout, 4), layout, 4);
    let threw = false;
    try { PC2.decodePointCloud2(msg); } catch (e) { threw = true; }
    if (!threw) throw new Error("expected throw for missing xyz");
  });

  function assertEqual(a, b) {
    if (a !== b) throw new Error(`expected ${b}, got ${a}`);
  }
  function assertClose(arr, expected, tol) {
    tol = tol || 1e-5;
    if (arr.length !== expected.length) {
      throw new Error(`length mismatch: ${arr.length} vs ${expected.length}`);
    }
    for (let i = 0; i < arr.length; i++) {
      if (Math.abs(arr[i] - expected[i]) > tol) {
        throw new Error(`index ${i}: ${arr[i]} vs ${expected[i]}`);
      }
    }
  }

  function run() {
    const results = [];
    for (const t of tests) {
      try {
        t.fn();
        results.push({ name: t.name, ok: true });
      } catch (e) {
        results.push({ name: t.name, ok: false, error: String(e && e.message || e) });
      }
    }
    return results;
  }

  return { tests, run };
});
