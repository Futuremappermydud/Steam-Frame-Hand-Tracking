import collections
import hashlib
import sys
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
from onnxconverter_common import float16

src, out = Path(sys.argv[1]), Path(sys.argv[2])
out.mkdir(parents=True, exist_ok=True)
names = ['grayscale_detection_160x160.onnx', 'grayscale_keypoint_jan18.onnx']
for name in names:
    model = onnx.load(src / name)
    g = model.graph
    ops = collections.Counter(n.op_type for n in g.node)
    params = sum(int(np.prod(t.dims)) for t in g.initializer)
    io = lambda vs: [(v.name, [d.dim_value or d.dim_param for d in v.type.tensor_type.shape.dim],
                      onnx.TensorProto.DataType.Name(v.type.tensor_type.elem_type)) for v in vs]
    print(f'== {name}: opset {[o.version for o in model.opset_import]}, {len(g.node)} nodes, {params/1e6:.2f} M params')
    print('   inputs ', io(g.input))
    print('   outputs', io(g.output))
    print('   ops    ', dict(ops.most_common()))
    fp16 = float16.convert_float_to_float16(model, keep_io_types=True)
    target = out / name
    onnx.save(fp16, target)
    print(f'   fp16 -> {target.name} sha256 {hashlib.sha256(target.read_bytes()).hexdigest()}')
    a = ort.InferenceSession(str(src / name), providers=['CPUExecutionProvider'])
    b = ort.InferenceSession(str(target), providers=['CPUExecutionProvider'])
    feeds = {}
    rng = np.random.default_rng(0)
    for v in a.get_inputs():
        shape = [d if isinstance(d, int) else 1 for d in v.shape]
        feeds[v.name] = rng.random(shape, dtype=np.float32)
    ra, rb = a.run(None, feeds), b.run(None, feeds)
    for o, x, y in zip(a.get_outputs(), ra, rb):
        print(f'   output {o.name}: max |fp32-fp16| {np.max(np.abs(x - y)):.4g} (fp32 range {np.min(x):.3g}..{np.max(x):.3g})')
