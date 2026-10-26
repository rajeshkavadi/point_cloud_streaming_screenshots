// PointCloud2 decoder — pure JS, no DOM dependencies.
// Works in the browser (window.PC2) and under Node (module.exports).
//
// decodePointCloud2(msg) accepts a sensor_msgs/PointCloud2 as delivered by
// rosbridge (JSON with base64 `data`) or a plain object with a Uint8Array
// `data`, and returns:
//   { positions: Float32Array(3*n), intensity: Float32Array(n)|null, count: n }
// XYZ are located via the message field descriptors (offsets/datatypes) —
// never assumed to be contiguous float32. Non-finite points are skipped.

(function (root, factory) {
  if (typeof module === "object" && module.exports) {
    module.exports = factory();
  } else {
    root.PC2 = factory();
  }
})(typeof self !== "undefined" ? self : this, function () {
  "use strict";

  // sensor_msgs/PointField datatype constants
  const INT8 = 1, UINT8 = 2, INT16 = 3, UINT16 = 4,
        INT32 = 5, UINT32 = 6, FLOAT32 = 7, FLOAT64 = 8;

  const DATATYPE_SIZE = {
    [INT8]: 1, [UINT8]: 1, [INT16]: 2, [UINT16]: 2,
    [INT32]: 4, [UINT32]: 4, [FLOAT32]: 4, [FLOAT64]: 8,
  };

  function base64ToBytes(b64) {
    const bin = atob(b64);
    const out = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
    return out;
  }

  function readScalar(view, offset, datatype, littleEndian) {
    switch (datatype) {
      case INT8: return view.getInt8(offset);
      case UINT8: return view.getUint8(offset);
      case INT16: return view.getInt16(offset, littleEndian);
      case UINT16: return view.getUint16(offset, littleEndian);
      case INT32: return view.getInt32(offset, littleEndian);
      case UINT32: return view.getUint32(offset, littleEndian);
      case FLOAT32: return view.getFloat32(offset, littleEndian);
      case FLOAT64: return view.getFloat64(offset, littleEndian);
      default: return NaN;
    }
  }

  function findField(fields, names) {
    for (const n of names) {
      const f = fields.find((f) => f.name === n);
      if (f) return f;
    }
    return null;
  }

  /**
   * Decode a PointCloud2 message into flat Float32 arrays.
   * @param {object} msg  rosbridge/plain PointCloud2
   * @param {object} [opts]
   * @param {number} [opts.maxPoints]  hard cap; uniform stride subsample above it
   * @returns {{positions: Float32Array, intensity: Float32Array|null,
   *            count: number, total: number, skipped: number}}
   */
  function decodePointCloud2(msg, opts) {
    opts = opts || {};
    const fields = msg.fields || [];
    const pointStep = msg.point_step | 0;
    const rowStep = msg.row_step | 0;
    const width = msg.width | 0;
    const height = (msg.height | 0) || 1;
    const littleEndian = !msg.is_bigendian;

    let bytes;
    if (msg.data instanceof Uint8Array) {
      bytes = msg.data;
    } else if (typeof msg.data === "string") {
      bytes = base64ToBytes(msg.data);
    } else if (Array.isArray(msg.data)) {
      bytes = Uint8Array.from(msg.data);
    } else {
      throw new Error("unsupported PointCloud2.data encoding");
    }

    const fx = findField(fields, ["x"]);
    const fy = findField(fields, ["y"]);
    const fz = findField(fields, ["z"]);
    if (!fx || !fy || !fz) {
      throw new Error("PointCloud2 message lacks x/y/z fields: " +
        fields.map((f) => f.name).join(","));
    }
    const fi = findField(fields, ["intensity", "i"]);

    const totalPoints = width * height;
    if (pointStep <= 0) throw new Error("invalid point_step: " + pointStep);

    // Stride subsample before decoding when over cap.
    let stride = 1;
    if (opts.maxPoints && totalPoints > opts.maxPoints) {
      stride = Math.ceil(totalPoints / opts.maxPoints);
    }
    const maxOut = Math.min(totalPoints, Math.ceil(totalPoints / stride));

    const positions = new Float32Array(maxOut * 3);
    const intensity = fi ? new Float32Array(maxOut) : null;
    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);

    let n = 0, skipped = 0;
    for (let row = 0; row < height; row++) {
      const rowBase = row * rowStep;
      for (let col = 0; col < width; col += stride) {
        const base = rowBase + col * pointStep;
        if (base + pointStep > bytes.byteLength) break; // truncated message
        const x = readScalar(view, base + fx.offset, fx.datatype, littleEndian);
        const y = readScalar(view, base + fy.offset, fy.datatype, littleEndian);
        const z = readScalar(view, base + fz.offset, fz.datatype, littleEndian);
        if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
          skipped++;
          continue;
        }
        positions[n * 3] = x;
        positions[n * 3 + 1] = y;
        positions[n * 3 + 2] = z;
        if (intensity) {
          intensity[n] = readScalar(view, base + fi.offset, fi.datatype, littleEndian);
        }
        n++;
      }
    }
    return { positions, intensity, count: n, total: totalPoints, skipped };
  }

  return { decodePointCloud2, base64ToBytes, DATATYPE_SIZE };
});
